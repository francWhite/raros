import express from 'express';
import process from 'process';
import dotenv from 'dotenv';
import { controllerRouter } from './controller.routes';

dotenv.config();

const app = express();
const port = process.env.PORT || 8000;

app.use('/api/controller', controllerRouter);

app.listen(port, () => {
  console.log(`Server is Fire at http://localhost:${port}`);
});
