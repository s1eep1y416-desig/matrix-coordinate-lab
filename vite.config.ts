import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';

export default defineConfig({
  base: process.env.GITHUB_PAGES === 'true' ? '/matrix-coordinate-lab/' : '/',
  plugins: [react()],
  build: { outDir: 'dist', emptyOutDir: true },
});
