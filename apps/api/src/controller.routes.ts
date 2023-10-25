import express from 'express';

export const controllerRouter = express.Router();
controllerRouter.use(express.json());

controllerRouter.get('/', async (req, res) => {
  res.send('Hello World!');
});

controllerRouter.post('/led', async (req, res) => {
  res.send('OK');
});
