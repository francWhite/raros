import express from 'express';
import { Request, Response } from 'express';
import { rosService } from '../ros.service';
import { decode } from 'uuid-base64-ts';
import { ActionInvocationResult } from '../ros.model';

export const navigationRouter = express.Router();
navigationRouter.use(express.json());

navigationRouter.post('/stop', async (req: Request<undefined, undefined, undefined>, res: Response) => {
  console.log(req.originalUrl);

  rosService.callService(
    '/raros/navigation/stop',
    'std_srvs/srv/Empty',
    {},
    () => res.send(),
    (error) => res.status(500).send(error),
  );
});

navigationRouter.post('/move', async (req: Request<undefined, undefined, MoveRequest>, res: Response) => {
  console.log(req.originalUrl, req.body);

  const requestData = {
    distance: req.body.distance,
    speed: req.body.endSpeed,
    speed_start: req.body.startSpeed,
    direction: { value: req.body.direction } };
  rosService.callService(
    '/raros/action_api/navigation/move',
    'raros_interfaces/srv/ActionMove',
    requestData,
    (response: ActionInvocationResult) => {
      const uidString = decode(response.goal_id.uuid);
      res.json({ goal_id: uidString }).send();
    },
    (error) => res.status(500).send(error),
  );
});

navigationRouter.post('/rotate', async (req: Request<undefined, undefined, RotateRequest>, res: Response) => {
  console.log(req.originalUrl, req.body);

  const requestData = { ...req.body, direction: { value: req.body.direction } };
  rosService.callService(
    '/raros/action_api/navigation/rotate',
    'raros_interfaces/srv/ActionRotate',
    requestData,
    (response: ActionInvocationResult) => {
      const uidString = decode(response.goal_id.uuid);
      res.json({ goal_id: uidString }).send();
    },
    (error) => res.status(500).send(error),
  );
});

navigationRouter.post('/turn', async (req: Request<undefined, undefined, TurnRequest>, res: Response) => {
  console.log(req.originalUrl, req.body);

  const requestData = { ...req.body, direction: { value: req.body.direction } };
  rosService.callService(
    '/raros/action_api/navigation/turn',
    'raros_interfaces/srv/ActionTurn',
    requestData,
    (response: ActionInvocationResult) => {
      const uidString = decode(response.goal_id.uuid);
      res.json({ goal_id: uidString }).send();
    },
    (error) => res.status(500).send(error),
  );
});

type MoveRequest = {
  distance: number;
  startSpeed: number;
  endSpeed: number;
  direction: number;
};

type RotateRequest = {
  angle: number;
  direction: number;
};

type TurnRequest = {
  angle: number;
  radius: number;
  direction: number;
};
