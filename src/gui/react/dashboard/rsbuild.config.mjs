import { defineConfig } from '@rsbuild/core';
import { pluginReact } from '@rsbuild/plugin-react';

export default defineConfig({
  plugins: [pluginReact()],
	server: {
    cors: {
      origin: '0.0.0.0',
    },
  },
});
