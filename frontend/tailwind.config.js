// tailwind.config.js
/** @type {import('tailwindcss').Config} */
export default {
  content: [
    './index.html',
    './src/**/*.{js,ts,jsx,tsx}',
  ],
  theme: {
    extend: {
      colors: {
        app: {
          dark: '#1E1E1E',
          card: '#2A2A2E',
          blue: '#0088FF',
          green: '#4CAF50',
          warning: '#FF3B30',
        },
      },
      fontFamily: {
        sans: ['"Pretendard"', 'ui-sans-serif', 'system-ui'],
        mono: ['"Fira Code"', 'monospace'],
      },
    },
  },
  plugins: [],
}
