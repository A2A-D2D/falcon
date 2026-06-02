// Build a Falcon-512 raw signature image from an RTL s2 dump.
// Input s2 words are 32 lines of 256-bit hex, 16 little-endian int16 lanes
// per word. Output words are 256-bit little-endian byte lanes for direct
// loading at VERIFY_SIG_BASE.

const fs = require("fs");

const inFile = process.argv[2] || "tb/rtl_s2_i16.hex";
const outFile = process.argv[3] || "tb/rtl_raw_sig.hex";
const nonceHex = process.argv[4] || "";

function readS2Words(name) {
  const text = fs.readFileSync(name, "utf8");
  return text.split(/\r?\n/).map(s => s.trim()).filter(Boolean);
}

function int16FromHex(v) {
  const x = Number(v & 0xffffn);
  return x >= 0x8000 ? x - 0x10000 : x;
}

function pushBit(bits, b) {
  bits.push(b ? 1 : 0);
}

function encodeCoeff(bits, x) {
  const sign = x < 0 ? 1 : 0;
  let abs = x < 0 ? -x : x;
  pushBit(bits, sign);
  for (let i = 6; i >= 0; i--) {
    pushBit(bits, (abs >> i) & 1);
  }
  abs = abs >> 7;
  while (abs > 0) {
    pushBit(bits, 0);
    abs--;
  }
  pushBit(bits, 1);
}

function parseNonce(hex) {
  if (!hex) return new Array(40).fill(0);
  const clean = hex.replace(/^0x/, "").replace(/[^0-9a-fA-F]/g, "");
  if (clean.length !== 80) {
    throw new Error("nonce hex must contain exactly 40 bytes");
  }
  const out = [];
  for (let i = 0; i < 40; i++) {
    out.push(parseInt(clean.slice(i * 2, i * 2 + 2), 16));
  }
  return out;
}

const words = readS2Words(inFile);
if (words.length < 32) {
  throw new Error(`expected 32 s2 words in ${inFile}, got ${words.length}`);
}

const coeffs = [];
for (let w = 0; w < 32; w++) {
  const word = BigInt("0x" + words[w]);
  for (let lane = 0; lane < 16; lane++) {
    coeffs.push(int16FromHex((word >> BigInt(lane * 16)) & 0xffffn));
  }
}

const bytes = [0x29].concat(parseNonce(nonceHex));
const bits = [];
for (const c of coeffs) encodeCoeff(bits, c);

for (let i = 0; i < bits.length; i += 8) {
  let b = 0;
  for (let j = 0; j < 8; j++) {
    b = (b << 1) | (bits[i + j] || 0);
  }
  bytes.push(b);
}

while ((bytes.length & 31) !== 0) bytes.push(0);
while (bytes.length < 704) bytes.push(0);

const lines = [];
for (let off = 0; off < bytes.length; off += 32) {
  let s = "";
  for (let i = 31; i >= 0; i--) {
    s += bytes[off + i].toString(16).padStart(2, "0");
  }
  lines.push(s);
}

fs.writeFileSync(outFile, lines.join("\n") + "\n");
console.log(`RAW_SIG words=${lines.length} bytes=${bytes.length} input=${inFile} output=${outFile}`);
