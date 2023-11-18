import express from 'express';
import process from 'process';
import dotenv from 'dotenv';
import { controllerRouter } from './routes/controller.routes';
import { magnetRouter } from './routes/magnet.routes';
import { rosService } from './ros.service';
import { buzzerRouter } from './routes/buzzer.routes';
import { actionRouter } from './routes/action.routes';
import { colorRouter } from './routes/color.routes';

dotenv.config();

const app = express();
const port = process.env.PORT || 8000;

app.use('/api/controller', controllerRouter);
app.use('/api/magnet', magnetRouter);
app.use('/api/buzzer', buzzerRouter);
app.use('/api/actions', actionRouter);
app.use('/api/color', colorRouter);

rosService.connect();

app.listen(port, () => {
  console.log(`Server is Fire at http://localhost:${port}`);
});
