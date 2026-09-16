import { defineConfig } from 'vitest/config';
import react from '@vitejs/plugin-react';

export default defineConfig({
  plugins: [react()],
  build: {
    rollupOptions: {
      input: { main: 'index.html', sentry: 'sentry.html' }
    }
  },
  test: {
    environment: 'node',
    globals: true
  }
});
