import { fileURLToPath, URL } from 'node:url'

import { defineConfig } from 'vite'
import vue from '@vitejs/plugin-vue'
import vueDevTools from 'vite-plugin-vue-devtools'
import { VitePWA } from 'vite-plugin-pwa'
import { templateCompilerOptions } from '@tresjs/core'

// The backend (Crow) listens on 9002. In development Vite forwards /api and /ws to it,
// so the browser sees one origin and no CORS setup is needed. In Docker, Caddy does the same.
const backend = process.env.VCORTEX_BACKEND ?? 'http://localhost:9002'

// https://vite.dev/config/
export default defineConfig({
  plugins: [
    // TresJS: tell the Vue compiler that <TresMesh>, <TresGroup>, ... are three.js objects.
    vue({ ...templateCompilerOptions }),
    vueDevTools(),
    VitePWA({
      registerType: 'autoUpdate',
      // Favicon and icons are generated from public/logo.svg (see pwa-assets.config.ts).
      pwaAssets: { config: true },
      manifest: {
        name: 'vCortex',
        short_name: 'vCortex',
        description: 'Live digital twin of a building from a lidar point cloud.',
        theme_color: '#0f172a',
        background_color: '#0f172a',
        display: 'standalone',
      },
      workbox: {
        globPatterns: ['**/*.{js,css,html,svg,png,ico,woff2}'],
        // /api and /ws are live data from the backend: never answer them from the cache.
        navigateFallbackDenylist: [/^\/api\//, /^\/ws\//],
      },
    }),
  ],
  resolve: {
    alias: {
      '@': fileURLToPath(new URL('./src', import.meta.url)),
    },
  },
  server: {
    proxy: {
      '/api': { target: backend, changeOrigin: true },
      '/ws': { target: backend, changeOrigin: true, ws: true },
    },
  },
})
