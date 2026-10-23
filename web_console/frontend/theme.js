// Apply the saved theme before painting. Default to the device preference.
(() => {
  let saved = null;
  try { saved = localStorage.getItem("g1_theme"); } catch { /* Storage may be unavailable. */ }
  const theme = saved === "light" || saved === "dark" ? saved : matchMedia("(prefers-color-scheme: dark)").matches ? "dark" : "light";
  document.documentElement.dataset.theme = theme;
  document.querySelector('meta[name="theme-color"]').content = theme === "dark" ? "#0b1730" : "#eef1f5";
})();
