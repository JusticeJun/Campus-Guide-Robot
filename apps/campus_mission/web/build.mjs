import {build} from 'esbuild';
import {mkdir,copyFile} from 'node:fs/promises';
await mkdir('dist',{recursive:true});
await build({entryPoints:['src/main.jsx'],bundle:true,minify:true,outfile:'dist/app.js',jsx:'automatic',define:{'process.env.NODE_ENV':'"production"'},legalComments:'linked'});
await copyFile('src/index.html','dist/index.html');
await copyFile('src/style.css','dist/style.css');
