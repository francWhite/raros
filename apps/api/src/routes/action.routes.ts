import express from 'express';
import { Request, Response } from 'express';
import { rosService } from '../ros.service';

export const actionRouter = express.Router();
actionRouter.use(express.json());

actionRouter.get(
  '/goals/:goalId',
  async (req: Request<{ goalId: string }, undefined, ActionCompletedResult>, res: Response) => {
    const requestData = { goal_id: req.params.goalId };
    rosService.callService(
      '/raros/action_api/action_completed',
      'raros_interfaces/srv/ActionCompleted',
      requestData,
      (response: ActionCompletedResult) =>
        res
          .json({
            id: req.query.goal_id,
            completed: response.completed,
          })
          .send(),
      (error) => res.status(500).send(error),
    );
  },
);
type ActionCompletedResult = {
  completed: boolean;
};
