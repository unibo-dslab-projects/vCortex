// the only script that may name quarkdown's DOM; components ask here for elements,
// so a quarkdown upgrade that renames them touches this file and style/quarkdown.css only.
window.VC = {
  ready: function (fn) {
    if (document.readyState === "loading") {
      document.addEventListener("DOMContentLoaded", fn);
    } else {
      fn();
    }
  },
  rootPath: function () {
    return document.querySelector('meta[name="quarkdown:root-path"]').content;
  },
  content: function () {
    return document.querySelector(".content-wrapper > main");
  },
  sidebar: function () {
    return document.querySelector(".content-wrapper > aside:first-child");
  },
  pageList: function () {
    return document.querySelector('nav[data-role="page-list"]');
  },
  diagrams: function () {
    return Array.prototype.slice.call(document.querySelectorAll(".mermaid > svg:not([data-sketched])"));
  },
  // the sidebar heading above the link to this page, or null on pages outside the nav
  pageSection: function () {
    var nav = this.pageList();
    if (!nav) return null;
    var here = this.normalize(location.pathname);
    var links = nav.querySelectorAll("a");
    for (var i = 0; i < links.length; i++) {
      var there = new URL(links[i].getAttribute("href"), location.href).pathname;
      if (this.normalize(there) !== here) continue;
      var list = links[i].closest("ul,ol");
      var heading = list && list.previousElementSibling;
      return heading ? heading.textContent.trim() : null;
    }
    return null;
  },
  normalize: function (path) {
    return path.replace(/\/index\.html$/, "").replace(/\/$/, "");
  },
};
