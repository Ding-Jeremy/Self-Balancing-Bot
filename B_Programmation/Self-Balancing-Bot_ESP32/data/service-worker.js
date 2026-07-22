const cacheName = 'self-balancing-bot-cache-v2';
const filesToCache = [
  '/',
  '/index.html',
  '/style.css',
  '/script.js',
  '/nipplejs.min.js',
  '/heig_vd_logo.png',
  '/icon-192.png',
  '/icon-512.png'
];

// Install service worker
self.addEventListener('install', (e) => {
  e.waitUntil(
    caches.open(cacheName).then((cache) => cache.addAll(filesToCache))
  );
});

self.addEventListener('activate', (event) => {
  event.waitUntil(
    caches.keys().then((cacheNames) => Promise.all(
      cacheNames
        .filter((name) => name.startsWith('self-balancing-bot-cache-') && name !== cacheName)
        .map((name) => caches.delete(name))
    ))
  );
});

// Fetch resources
self.addEventListener('fetch', (e) => {
  e.respondWith(
    caches.match(e.request).then((response) => {
      return response || fetch(e.request);
    })
  );
});
