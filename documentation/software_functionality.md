# Functionality

The following outlines notional functionality expected of our system as epxressed through a series of steps/actions that the system must take for transportation of each individual object. The actions required to facilitate transport of a single object are to be considered before the handling of additional objects.

1. Movement of a free compartment to become aligned with the conveyor
   1. Movement (planar and height) towards the conveyor pickup point
   2. Detection and stopping of movement when alignment is achieved
2. Logging of the type of object on the conveyor
3. Movement of the object into the compartment
   1. Push off/grab from conveyor
   2. Detection/confirmation of object presence in compartment
4. Movement of the occupied compartment towards the desired drop off location
   1. Movement (planar and height) towards the conveyor dropoff point
   2. Detection and stopping of movement when alignment is achieved
5. Movement of object from compartment to drop off location
   1. Push off/grab from compartment to dropoff
   2. Detection/confirmation of object no longer present in compartment

The above steps should be sufficient to ensure correct pickup/dropoff functionality for a single item. The addition of handling of multiple objects at the same time can be easily handled through the additional consideration of moving to a free compartment in step 1 while step 5 is completed when a particular object has reached its destination.