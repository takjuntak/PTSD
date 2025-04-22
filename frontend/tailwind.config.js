/** @type {import('tailwindcss').Config} */
export default {
  content: [
    "./index.html",
    "./src/**/*.{js,ts,jsx,tsx}",
  ],
  theme: {
    extend: {
      colors: {
        'app-dark': '#1E1E1E',
        'app-card': '#2A2A2A',
        'app-blue': '#0088FF',
        'app-green': '#4CAF50',
        'app-warning': '#FF3B30',
      },
    },
  },
  plugins: [],
}