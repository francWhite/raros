FROM node:20-alpine as build
WORKDIR /app
COPY package.json package-lock.json ./

RUN npm install

FROM node:20-alpine
LABEL org.opencontainers.image.source=https://github.com/francWhite/raros
LABEL org.opencontainers.image.description="raros_api"
LABEL org.opencontainers.image.licenses=MIT

WORKDIR /app
COPY --from=build /app /app
COPY . .
RUN npm run build

CMD ["npm", "run", "start"]