import express from 'express';
import { Request, Response } from 'express';
import { rosService } from '../ros.service';
import { decode } from 'uuid-base64-ts';
import { ActionInvocationResult } from '../ros.model';

export const buzzerRouter = express.Router();
buzzerRouter.use(express.json());

buzzerRouter.post('/', async (req: Request<undefined, undefined, Tone>, res: Response) => {
  console.log(req.originalUrl, req.body);

  const requestData = { ...req.body };
  rosService.callService(
    '/raros/action_api/buzzer/play_tone',
    'raros_interfaces/srv/ActionPlayTone',
    requestData,
    (response: ActionInvocationResult) => {
      const uidString = decode(response.goal_id.uuid);
      console.log('received goal_id', uidString);

      res.json({ goal_id: uidString }).send();
    },
    (error) => res.status(500).send(error),
  );
});

type Tone = {
  frequency: number;
  duration: number;
};
