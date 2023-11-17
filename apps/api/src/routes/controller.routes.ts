import express from 'express';
import { rosService } from '../ros.service';

export const controllerRouter = express.Router();
controllerRouter.use(express.json());

controllerRouter.get('/', async (req, res) => {
  res.send('Hello World!');
});

controllerRouter.post('/led', async (req, res) => {
  console.log(req.body);
  rosService.publishToTopic('/api_led', req.body['state']);
  res.send();
});
