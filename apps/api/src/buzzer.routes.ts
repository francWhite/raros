import express from 'express';
import { Request, Response } from 'express';
import { rosService } from './ros.service';

export const buzzerRouter = express.Router();
buzzerRouter.use(express.json());

buzzerRouter.post('/tone', async (req: Request<undefined, undefined, Tone>, res: Response) => {
  console.log(req.originalUrl, req.body);
  const payload = { ...req.body };
  rosService.callService('/buzzer/play_tone', 'raros_interfaces/srv/PlayTone', payload, () => {});
  res.send();
});

type Tone = {
  frequency: number;
  duration: number;
};
