import { fileURLToPath, URL } from 'node:url'

import { defineConfig } from 'vite'
import vue from '@vitejs/plugin-vue'
import vueDevTools from 'vite-plugin-vue-devtools'
import { VitePWA } from 'vite-plugin-pwa'
import { templateCompilerOptions } from '@tresjs/core'

// backend (crow) listens on 9002; vite (dev) and caddy (docker) forward /api and /ws to it:
// one origin, no cors
const backend = process.env.VCORTEX_BACKEND ?? 'http://localhost:9002'

// https://vite.dev/config/
export default defineConfig({
  plugins: [
    // tresjs: tells the vue compiler <TresMesh>, <TresGroup>, ... are three.js objects
    vue({ ...templateCompilerOptions }),
    vueDevTools(),
    VitePWA({
      registerType: 'autoUpdate',
      // favicon and icons generated from public/logo.svg (see pwa-assets.config.ts)
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
        // /api and /ws are live backend data: never served from cache
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
