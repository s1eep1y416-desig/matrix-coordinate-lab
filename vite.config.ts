import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';

export default defineConfig(({ mode }) => ({
  base: mode === 'github-pages' ? '/matrix-coordinate-lab/' : '/',
  plugins: [react()],
  build: { outDir: 'dist', emptyOutDir: true },
}));
