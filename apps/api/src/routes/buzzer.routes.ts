import express from 'express';
import { Request, Response } from 'express';
import { rosService } from '../ros.service';

export const buzzerRouter = express.Router();
buzzerRouter.use(express.json());

buzzerRouter.post('/tone', async (req: Request<undefined, undefined, Tone>, res: Response) => {
  console.log(req.originalUrl, req.body);
  const requestData = { ...req.body };
  rosService.sendGoal('/buzzer/play_tone', 'raros_interfaces/action/PlayTone', requestData, (status) =>
    console.log('status', status),
  );
  res.send();
});

type Tone = {
  frequency: number;
  duration: number;
};
