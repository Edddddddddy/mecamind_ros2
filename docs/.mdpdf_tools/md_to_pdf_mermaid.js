#!/usr/bin/env node
/**
 * Markdown → PDF with Mermaid rendering + CJK CSS.
 * Usage: node md_to_pdf_mermaid.js <input.md> [output.pdf]
 */
const fs = require('fs');
const path = require('path');
const { marked } = require('marked');
const puppeteer = require('puppeteer');

async function main() {
  const inMd = path.resolve(process.argv[2]);
  if (!inMd || !fs.existsSync(inMd)) {
    console.error('Usage: node md_to_pdf_mermaid.js <input.md> [output.pdf]');
    process.exit(1);
  }
  const outPdf = path.resolve(
    process.argv[3] || inMd.replace(/\.md$/i, '.pdf')
  );
  const cssPath = path.join(__dirname, 'cjk.css');
  const mermaidPath = require.resolve('mermaid/dist/mermaid.min.js');
  const chrome =
    process.env.PUPPETEER_EXECUTABLE_PATH ||
    '/home/lqf/.cache/puppeteer/chrome/linux-150.0.7871.24/chrome-linux64/chrome';

  const md = fs.readFileSync(inMd, 'utf8');
  const css = fs.readFileSync(cssPath, 'utf8');
  const mermaidJs = fs.readFileSync(mermaidPath, 'utf8');

  const renderer = new marked.Renderer();
  const defaultCode = renderer.code.bind(renderer);
  renderer.code = (token) => {
    const lang = (token.lang || '').trim();
    const text = token.text || '';
    if (lang === 'mermaid') {
      // Put source in data-* so auto-start / prior render cannot pollute textContent.
      const encoded = Buffer.from(text, 'utf8').toString('base64');
      return `<div class="mermaid" data-mermaid-b64="${encoded}"></div>\n`;
    }
    return defaultCode(token);
  };
  marked.use({ renderer });

  const bodyHtml = marked.parse(md, { async: false });

  const html = `<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8"/>
<title>${path.basename(inMd)}</title>
<style>
${css}
.mermaid {
  text-align: center;
  margin: 0.6em auto;
  page-break-inside: avoid;
  overflow: visible;
  max-width: 100%;
}
.mermaid svg {
  max-width: 100% !important;
  height: auto !important;
  display: inline-block;
}
</style>
</head>
<body>
${bodyHtml}
<script>${mermaidJs}</script>
<script>mermaid.initialize({ startOnLoad: false, securityLevel: 'loose' });</script>
</body>
</html>`;

  const tmpHtml = path.join(
    path.dirname(outPdf),
    `.tmp_${path.basename(outPdf, '.pdf')}.html`
  );
  fs.writeFileSync(tmpHtml, html, 'utf8');

  const browser = await puppeteer.launch({
    executablePath: chrome,
    headless: 'new',
    args: ['--no-sandbox', '--disable-setuid-sandbox', '--font-render-hinting=none'],
  });
  try {
    const page = await browser.newPage();
    page.on('pageerror', (err) => console.error('pageerror:', err.message));
    await page.goto('file://' + tmpHtml, { waitUntil: 'load', timeout: 120000 });

    const renderResult = await page.evaluate(async () => {
      mermaid.initialize({
        startOnLoad: false,
        securityLevel: 'loose',
        theme: 'base',
        flowchart: {
          htmlLabels: true,
          curve: 'basis',
          useMaxWidth: true,
          nodeSpacing: 12,
          rankSpacing: 16,
          padding: 4,
        },
        themeVariables: {
          fontSize: '11px',
        },
      });
      const nodes = Array.from(document.querySelectorAll('.mermaid'));
      const errors = [];
      const b64decode = (b64) => {
        const bin = atob(b64);
        const bytes = Uint8Array.from(bin, (c) => c.charCodeAt(0));
        return new TextDecoder('utf-8').decode(bytes);
      };
      for (let i = 0; i < nodes.length; i++) {
        const b64 = nodes[i].getAttribute('data-mermaid-b64') || '';
        const code = b64 ? b64decode(b64).trim() : nodes[i].textContent.trim();
        try {
          const id = 'mmd-' + i;
          const out = await mermaid.render(id, code);
          nodes[i].innerHTML = out.svg;
        } catch (e) {
          const msg = e && e.message ? e.message : String(e);
          errors.push({ i, msg, head: code.slice(0, 80) });
          nodes[i].innerHTML =
            '<pre style="color:#b00020;text-align:left;white-space:pre-wrap;font-size:10px">' +
            'Mermaid error: ' + msg + '\n\n' + code + '</pre>';
        }
      }

      // Fit diagrams to page: use viewBox; keep readable, never page-tall.
      const MAX_W = 540;
      const MAX_H = 160;
      const sizes = [];
      for (const wrap of nodes) {
        const svg = wrap.querySelector('svg');
        if (!svg) continue;
        const vb = (svg.getAttribute('viewBox') || '').trim().split(/[\s,]+/).map(Number);
        let w = 0, h = 0;
        if (vb.length === 4 && vb[2] > 0 && vb[3] > 0) {
          w = vb[2];
          h = vb[3];
        } else {
          const rect = svg.getBoundingClientRect();
          w = rect.width;
          h = rect.height;
        }
        if (!w || !h) continue;
        // Prefer fitting width; cap height. Avoid crushing wide flowcharts into needles.
        let scale = Math.min(1.25, MAX_W / w);
        if (h * scale > MAX_H) scale = MAX_H / h;
        if (w * scale < MAX_W * 0.55 && h / w > 1.2) {
          // Tall diagram: still cap height but keep readable width floor
          scale = Math.max(scale, Math.min(MAX_W / w, MAX_H / h));
        }
        const nw = Math.max(1, Math.round(w * scale));
        const nh = Math.max(1, Math.round(h * scale));
        svg.setAttribute('width', String(nw));
        svg.setAttribute('height', String(nh));
        svg.style.width = nw + 'px';
        svg.style.height = nh + 'px';
        svg.style.maxWidth = '100%';
        wrap.style.margin = '0.45em auto';
        sizes.push({ view: [Math.round(w), Math.round(h)], out: [nw, nh], scale: Number(scale.toFixed(3)) });
      }
      return {
        total: nodes.length,
        ok: nodes.filter((n) => n.querySelector('svg')).length,
        errors,
        sizes,
      };
    });
    console.log('Mermaid render:', JSON.stringify(renderResult, null, 2));

    await page.pdf({
      path: outPdf,
      format: 'A4',
      printBackground: true,
      margin: { top: '16mm', right: '12mm', bottom: '16mm', left: '12mm' },
    });
  } finally {
    await browser.close();
    try { fs.unlinkSync(tmpHtml); } catch (_) {}
  }
  console.log('Wrote', outPdf);
}

main().catch((e) => {
  console.error(e);
  process.exit(1);
});
