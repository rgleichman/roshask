{-# LANGUAGE TemplateHaskell #-}
module Turtle (main) where
import Data.Complex
import Ros.Node
import Ros.Topic (cons, repeatM)
import Ros.Topic.Util (tee, filterBy, everyNew, interruptible, gate)
import Ros.Turtlesim.Pose
import Ros.Turtlesim.Velocity
import Ros.Logging
import System.IO (hFlush, stdout)

-- Setup: rosrun turtlesim turtlesim_node

-- A type synonym for a 2D point.
type Point = Complex Float

-- A Topic of user-supplied waypoint trajectories.
getTraj :: Topic IO [Point]
getTraj = repeatM (do putStr "Enter waypoints in the form [(x1,y1), (x2,y2) ... ]: " >> hFlush stdout
                      $(logInfo "Waiting for new traj")
                      (map (uncurry (:+)) . read) `fmap` getLine)

wrapAngle :: (Floating a, Ord a) => a -> a
wrapAngle theta 
  | theta < 0    = theta + 2 * pi
  | theta > 2*pi = theta - 2 * pi
  | otherwise    = theta

angleDiff :: (Floating a, Ord a) => a -> a -> a
angleDiff x y = wrapAngle (x' + pi - y') - pi
  where x' = if x < 0 then x + 2*pi else x
        y' = if y < 0 then y + 2*pi else y

-- Produce a unit value every time a goal is reached.
arrivalTrigger :: Topic IO Point -> Topic IO Pose -> Topic IO ()
arrivalTrigger goals poses = fmap (const ()) $
                             filterBy (fmap arrived goals) (fmap p2v poses)
  where arrived goal pose = magnitude (goal - pose) < 1.5
        p2v (Pose x y _ _ _) = x :+ y

-- Navigate to a goal given a current pose estimate.
navigate :: (Point, Pose) -> Velocity
navigate (goal, pos) = Velocity (min 2 (magnitude v)) angVel
  where v        = goal - (x pos :+ y pos)
        thetaErr = (angleDiff (phase v) (theta pos)) * (180 / pi)
        angVel   = signum thetaErr * (min 2 (abs thetaErr))

main :: IO ()
main = runNode "HaskellBTurtle" $
       do enableLogging (Just Warn)
          poses <- subscribe "/turtle1/pose"
          (t1,t2) <- liftIO . tee . interruptible $ getTraj
          let goals = gate t1 (cons () (arrivalTrigger t2 poses))
          advertise "/turtle1/command_velocity" $
                    fmap navigate (everyNew goals poses)
