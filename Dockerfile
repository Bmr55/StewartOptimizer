FROM nginx:stable-alpine

COPY index.html /usr/share/nginx/html/
COPY src/ /usr/share/nginx/html/src/
COPY styles/ /usr/share/nginx/html/styles/
COPY assets/ /usr/share/nginx/html/assets/
COPY examples/ /usr/share/nginx/html/examples/
