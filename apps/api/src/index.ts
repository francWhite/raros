import express from 'express';
import process from 'process';
import dotenv from 'dotenv';
import { controllerRouter } from './controller.routes';
import { magnetRouter } from './magnet.routes';
import { rosService } from './ros.service';

dotenv.config();

const app = express();
const port = process.env.PORT || 8000;

app.use('/api/controller', controllerRouter);
app.use('/api/magnet', magnetRouter);

rosService.connect();

app.listen(port, () => {
  console.log(`Server is Fire at http://localhost:${port}`);
});
