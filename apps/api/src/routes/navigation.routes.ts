import express from 'express';
import { Request, Response } from 'express';
import { rosService } from '../ros.service';
import { decode } from 'uuid-base64-ts';
import { ActionInvocationResult } from '../ros.model';

export const navigationRouter = express.Router();
navigationRouter.use(express.json());

navigationRouter.post('/move', async (req: Request<undefined, undefined, MoveRequest>, res: Response) => {
  console.log(req.originalUrl, req.body);

  const requestData = { ...req.body };
  rosService.callService(
    '/raros/action_api/navigation/move',
    'raros_interfaces/srv/ActionMove',
    requestData,
    (response: ActionInvocationResult) => {
      const uidString = decode(response.goal_id.uuid);
      console.log('received goal_id', uidString);

      res.json({ goal_id: uidString }).send();
    },
    (error) => res.status(500).send(error),
  );
});

type MoveRequest = {
  distance: number;
  speed: number;
  direction: number;
};
