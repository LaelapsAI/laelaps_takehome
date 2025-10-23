import type { NextConfig } from "next";

const nextConfig: NextConfig = {
  async rewrites() {
    return [
      { source: '/status', destination: 'http://backend-node:8080/status' },
      { source: '/event', destination: 'http://backend-node:8080/event' },
      { source: '/events/:id/assign', destination: 'http://backend-node:8080/events/:id/assign' },
    ];
  },
};

export default nextConfig;
