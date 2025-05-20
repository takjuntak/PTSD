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
      keyframes: {
        breath: {
          '0%, 100%': { transform: 'scale(1)' },
          '50%': { transform: 'scale(1.06)' },
        },
      },
      animation: {
        breath: 'breath 1.2s ease-in-out infinite',
      },
    },
  },
  plugins: [],
}
