import { fileURLToPath, URL } from 'node:url'
import { defineConfig } from 'vite'
import { templateCompilerOptions } from '@tresjs/core'
import vue from '@vitejs/plugin-vue'
import vueDevTools from 'vite-plugin-vue-devtools'

// https://vite.dev/config/
export default defineConfig({
  plugins: [
    vue({ ...templateCompilerOptions }),
  ],

  assetsInclude: ["**/*.glb"],

  resolve: {
    alias: {
      '@': fileURLToPath(new URL('./src', import.meta.url)),
    },
  },

  build: {
    outDir: 'dist',
    assetsDir: 'assets',
    rollupOptions: {
      output: {
        // keep all JS chunks in /js
        entryFileNames: 'js/[name]-[hash].js',
        chunkFileNames: 'js/[name]-[hash].js',

        // place CSS, images, and other assets in their own folders
        assetFileNames: ({ name }) => {
          if (/\.(gif|jpe?g|png|svg|webp|ico)$/.test(name ?? '')) {
            return 'images/[name]-[hash][extname]'
          }
          if (/\.css$/.test(name ?? '')) {
            return 'css/[name]-[hash][extname]'
          }
          return 'assets/[name]-[hash][extname]'
        },
      },
    },
  },
})
