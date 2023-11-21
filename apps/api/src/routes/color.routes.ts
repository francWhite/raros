import express, { Request, Response } from 'express';
import { rosService } from '../ros.service';

export const colorRouter = express.Router();
colorRouter.use(express.json());

colorRouter.get('/', async (req: Request<undefined, undefined, undefined>, res: Response) => {
  console.log(req.originalUrl);

  rosService.readSingleMessageFromTopic(
    '/raros/color_sensor/color',
    'std_msgs/msg/ColorRGBA',
    (message: ColorRGBA) =>
      res
        .json({
          red: message.r,
          green: message.g,
          blue: message.b,
        })
        .send(),
    (error) => res.status(500).send(error),
  );
});

type ColorRGBA = {
  r: number;
  g: number;
  b: number;
  a: number;
};
