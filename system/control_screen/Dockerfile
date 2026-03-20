FROM node:20-alpine

# Ops spawns run_perception_rpicam.sh with bash (Alpine ships ash only by default).
RUN apk add --no-cache bash

WORKDIR /app

COPY package.json package-lock.json* ./
RUN npm install --production

COPY server.js ./
COPY public ./public

ENV PORT=3000
EXPOSE 3000

CMD ["node", "server.js"]
