import { readFile, writeFile } from 'node:fs/promises';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const projectRoot = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const distDir = path.join(projectRoot, 'dist');
const outputPath = path.join(projectRoot, 'Matrix-Coordinate-Lab-offline.html');

const mimeTypes = {
  '.svg': 'image/svg+xml',
  '.ttf': 'font/ttf',
  '.woff': 'font/woff',
  '.woff2': 'font/woff2',
};

const assetPath = (url) => {
  const relativePath = url.replace(/^\/+/, '');
  const resolved = path.resolve(distDir, relativePath);
  if (!resolved.startsWith(`${distDir}${path.sep}`)) {
    throw new Error(`Refusing to inline an asset outside dist/: ${url}`);
  }
  return resolved;
};

const asDataUrl = async (url) => {
  const filePath = assetPath(url);
  const mime = mimeTypes[path.extname(filePath).toLowerCase()] ?? 'application/octet-stream';
  const data = await readFile(filePath);
  return `data:${mime};base64,${data.toString('base64')}`;
};

let html = await readFile(path.join(distDir, 'index.html'), 'utf8');

const scriptMatch = html.match(/<script\b[^>]*\bsrc="([^"]+)"[^>]*><\/script>/);
const styleMatch = html.match(/<link\b[^>]*\brel="stylesheet"[^>]*\bhref="([^"]+)"[^>]*>/);

if (!scriptMatch || !styleMatch) {
  throw new Error('Could not find the built JavaScript and stylesheet in dist/index.html.');
}

let css = await readFile(assetPath(styleMatch[1]), 'utf8');
const cssAssetUrls = [...new Set([...css.matchAll(/url\((['"]?)(\/assets\/[^)'"\s]+)\1\)/g)].map((match) => match[2]))];

for (const url of cssAssetUrls) {
  css = css.split(url).join(await asDataUrl(url));
}

let javascript = await readFile(assetPath(scriptMatch[1]), 'utf8');
const javascriptAssetUrls = [...new Set(javascript.match(/\/assets\/[A-Za-z0-9_.-]+/g) ?? [])];
for (const url of javascriptAssetUrls) {
  javascript = javascript.split(url).join(await asDataUrl(url));
}
javascript = javascript.replaceAll('</script', '<\\/script');

html = html
  .replace(scriptMatch[0], () => `<script type="module">${javascript}</script>`)
  .replace(styleMatch[0], () => `<style>${css}</style>`);

const faviconMatch = html.match(/<link\b[^>]*\brel="icon"[^>]*\bhref="([^"]+)"[^>]*>/);
if (faviconMatch) {
  const faviconDataUrl = await asDataUrl(faviconMatch[1]);
  html = html.replace(faviconMatch[0], () => `<link rel="icon" href="${faviconDataUrl}">`);
}

const remainingAssetReferences = [
  ...html.matchAll(/\b(?:src|href)="\/assets\/[^\"]+/g),
  ...html.matchAll(/url\(\/?assets\/[^)]+/g),
].map((match) => match[0]);
if (remainingAssetReferences.length > 0) {
  throw new Error(`The generated offline page still contains external asset references: ${remainingAssetReferences.slice(0, 3).join(', ')}`);
}

await writeFile(outputPath, html);
console.log(`Offline page written to ${outputPath}`);
