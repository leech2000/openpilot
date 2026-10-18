import { readFileSync, writeFileSync } from 'node:fs';
import { createHash } from 'node:crypto';

const [templatePath, outputPath] = process.argv.slice(2);
if (!templatePath || !outputPath) {
  throw new Error('Usage: node build.mjs TEMPLATE OUTPUT');
}

const expectedTemplateSha = '45680281752c624aa1818cc39141b2c1c846b599';
const source = readFileSync(templatePath);
const gitSha = (data) => createHash('sha1').update(`blob ${data.length}\0`).update(data).digest('hex');
if (gitSha(source) !== expectedTemplateSha) throw new Error('Unexpected installer template');
if (source.subarray(0, 4).toString('hex') !== '7f454c46') throw new Error('Template is not ELF');
if (source.readUInt16LE(18) !== 183) throw new Error('Template is not AArch64');

const output = Buffer.from(source);
function replaceOne(placeholder, value, padding) {
  const needle = Buffer.from(placeholder);
  const replacement = Buffer.from(value);
  if (replacement.length > needle.length) throw new Error('Value exceeds slot');
  const offset = output.indexOf(needle);
  if (offset < 0 || output.indexOf(needle, offset + 1) !== -1) {
    throw new Error('Expected exactly one placeholder: ' + placeholder.slice(0, 16));
  }
  output.fill(padding, offset, offset + needle.length);
  replacement.copy(output, offset);
}

replaceOne('27182818284590452353602874713526624977572470936999595', 'leech2000/openpilot.git', 0);
replaceOne('161803398874989484820458683436563811772030917980576286213544862270526046281890244970720720418939113748475408807538689175212663386222353693179318006076672635443338908659593958290563832266131992829026788067520876689250171169620703222104321626954862629631361', 'carrot-wip', 0);
replaceOne('314159265358979323846264338327950288419', 'Leech CarrotPilot', 0x20);
if (!output.includes(Buffer.from('https://github.com/leech2000/openpilot.git\0'))) {
  throw new Error('Git URL is missing after patch');
}
writeFileSync(outputPath, output);
console.log(JSON.stringify({ bytes: output.length, gitBlobSha: gitSha(output), sha256: createHash('sha256').update(output).digest('hex') }, null, 2));

