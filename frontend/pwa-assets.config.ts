import { defineConfig, minimal2023Preset } from '@vite-pwa/assets-generator/config'

// Builds the favicon and the PWA icons (64, 192, 512, maskable, apple-touch) from one SVG.
export default defineConfig({
  headLinkOptions: {
    preset: '2023',
  },
  preset: minimal2023Preset,
  images: ['public/logo.svg'],
})
