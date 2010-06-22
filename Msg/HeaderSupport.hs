-- |If a message type's first field is of type Header, its sequence
-- number is automatically incremented by the ROS Topic machinery.
module Msg.HeaderSupport where
import Header

class HasHeader a where
    getHeader :: a -> Header
    setSequence :: a -> Word32 -> a
