module Main (main) where

import qualified Data.Map as M
import Control.Monad
import Control.Monad.IO.Class
import Data.IORef
import Data.List (unzip4)
import Data.StateVar as SV
import System.Exit

import Graphics.UI.GLFW
import Graphics.Rendering.OpenGL.GL.CoordTrans
import Graphics.Rendering.OpenGL as GL
import qualified Physics.Hipmunk as H

type Time = Double

------------------------------------------------------------
-- Some constants and utils
------------------------------------------------------------

-- | Desired (and maximum) frames per second.
desiredFPS :: Int
desiredFPS = 60

-- | How much seconds a frame lasts.
framePeriod :: Time
framePeriod = 1 / toEnum desiredFPS

-- | How many steps should be done per frame.
frameSteps :: Double
frameSteps = 6

-- | Maximum number of steps per frame (e.g. if lots of frames get
--   dropped because the window was minimized)
maxSteps :: Int
maxSteps = 20

-- | How much time should pass in each step.
frameDelta :: H.Time
frameDelta = 3.33e-3

-- | How much slower should the slow mode be.
slowdown :: Double
slowdown = 10

-- | 0 :: GLfloat
zero :: GLfloat
zero = 0

-- | Asserts that an @IO@ action returns @True@, otherwise
--   fails with the given message.
assertTrue :: IO Bool -> String -> IO ()
assertTrue act msg = do {b <- act; when (not b) (fail msg)}

-- | Constructs a Vector.
(+:) :: H.CpFloat -> H.CpFloat -> H.Vector
(+:) = H.Vector
infix 4 +:



------------------------------------------------------------
-- State
------------------------------------------------------------

-- | Our current program state that will be passed around.
data State = State {
      stSpace    :: H.Space,
      stCarState :: CarState,
      stControls :: CarControls,
      stShapes   :: M.Map H.Shape (IO () {- Drawing -}
                                  ,IO () {- Removal -}),
      stCollisionsVar :: IORef [H.Position]
    }
data CarState = Stopped | GoingLeft | GoingRight deriving (Eq, Ord, Enum)
type Object = (H.Shape, (IO (), IO ()))
type CarControls = (CarState -> IO ())

-- | Our initial state.
initialState :: IO State
initialState = do
  -- The (empty) space
  space <- H.newSpace
  H.gravity space SV.$= 0 +: -230

  -- Default objects
  seesaw  <- buildSeesaw space
  ground  <- buildGround space
  (car,c) <- buildCar space

  -- Add a callback to Hipmunk to draw collisions
  collisionsVar <- newIORef []
  let handler = do ps <- H.points
                   liftIO $ modifyIORef collisionsVar (ps ++)
  H.setDefaultCollisionHandler space $
    H.Handler {H.beginHandler     = Nothing
              ,H.preSolveHandler  = Nothing
              ,H.postSolveHandler = Just handler
              ,H.separateHandler  = Nothing}

  -- Our state
  return $ State {stSpace    = space
                 ,stCarState = Stopped
                 ,stControls = c
                 ,stShapes   = M.fromList [seesaw, ground, car]
                 ,stCollisionsVar = collisionsVar}

-- | Builds the ground
buildGround :: H.Space -> IO Object
buildGround space = do
  static <- H.newBody H.infinity H.infinity
  H.position static SV.$= -330 +: 0
  let seg1type = H.LineSegment (50 +: -230) (610 +: -230) 1
  seg1   <- H.newShape static seg1type 0
  H.friction   seg1 SV.$= 1.0
  H.elasticity seg1 SV.$= 0.6
  H.spaceAdd space (H.Static seg1)
  return (seg1, (drawMyShape seg1 seg1type, return ()))

-- | Builds the seesaw.
buildSeesaw :: H.Space -> IO Object
buildSeesaw space = do
  ---- Support
  let supportV = [-15 +: -20, -5 +: 20, 5 +: 20, 15 +: -20]
      supportT = H.Polygon supportV
      supportM = 500
      supportI = H.momentForPoly supportM supportV 0
  supportB <- H.newBody supportM supportI
  H.position supportB SV.$= 0 +: 20-230
  supportS <- H.newShape supportB supportT 0
  H.friction   supportS SV.$= 2.0
  H.elasticity supportS SV.$= 0.1
  H.spaceAdd space supportB
  H.spaceAdd space supportS
  ----- Board
  let boardV = [-100 +: 1, 100 +: 1, 100 +: -1, -100 +: -1]
      boardT = H.Polygon boardV
      boardM = 10
      boardI = H.momentForPoly boardM boardV 0
  boardB <- H.newBody boardM boardI
  H.position boardB SV.$= 0 +: 40-230
  boardS <- H.newShape boardB boardT 0
  let setBoardProps shape = do
        H.friction   shape SV.$= 2.0
        H.elasticity shape SV.$= 0.1
  setBoardProps boardS
  H.spaceAdd space boardB
  H.spaceAdd space boardS
  boardS2 <- forM (zip boardV $ tail $ cycle boardV) $ \(v1,v2) -> do
    seg <- H.newShape boardB (H.LineSegment v1 v2 0.1) 0
    setBoardProps seg
    H.spaceAdd space seg
    return seg
  ----- Constraint
  seesawJoint <- H.newConstraint supportB boardB (H.Pin (0 +: 20) 0)
  H.spaceAdd space seesawJoint
  ----- Avoiding self-collisions
  forM_ (supportS : boardS : boardS2) $ \s -> do
    H.group s SV.$= 1
  ----- Removing and drawing
  let drawSeeSaw = do
        drawMyShape supportS supportT
        drawMyShape boardS boardT
  let removeSeeSaw = do
        H.spaceRemove space supportB
        H.spaceRemove space supportS
        H.spaceRemove space boardB
        H.spaceRemove space boardS
        forM_ boardS2 (H.spaceRemove space)
        H.spaceRemove space seesawJoint
  return (supportS, (drawSeeSaw, removeSeeSaw))

-- | Build a small car.
buildCar :: H.Space -> IO (Object, CarControls)
buildCar space = do
  ---- Bodywork
  let bodyworkV = [-25 +: -9, -17 +: 10, 17 +: 10, 25 +: -9]
      bodyworkT = H.Polygon bodyworkV
      bodyworkP = (-150 +: -90)
      bodyworkM = 40
      bodyworkI = H.momentForPoly bodyworkM bodyworkV 0
  bodyworkB <- H.newBody bodyworkM bodyworkI
  H.position bodyworkB SV.$= bodyworkP
  bodyworkS <- H.newShape bodyworkB bodyworkT 0
  H.friction   bodyworkS SV.$= 1.5
  H.elasticity bodyworkS SV.$= 0.1
  H.spaceAdd space bodyworkB
  H.spaceAdd space bodyworkS
  ---- Wheels
  let wheelR = 12 -- radius
      wheelM = 200  -- mass
      wheelT = H.Circle wheelR
      wheelI = H.momentForCircle wheelM (0, wheelR) 0
      wheelJ1 x = H.DampedSpring (x +: 0) (0 +: 0) 23 0 10  -- spring
      wheelJ2 x = H.Groove (x +: -23.5, x +: -27) (0 +: 0)  -- groove
  (wheelBs, wheelSs, wheelCs, wheelMs) <-
      fmap unzip4 $ forM [-25, 25] $ \x -> do
    -- Basic
    wheelB <- H.newBody wheelM wheelI
    H.position wheelB SV.$= (x +: -24) + bodyworkP
    wheelS <- H.newShape wheelB wheelT 0
    H.friction   wheelS SV.$= 2.0
    H.elasticity wheelS SV.$= 0.25
    -- Constraints
    wheelC1 <- H.newConstraint bodyworkB wheelB (wheelJ1 x)
    wheelC2 <- H.newConstraint bodyworkB wheelB (wheelJ2 x)
    H.spaceAdd space wheelB
    H.spaceAdd space wheelS
    H.spaceAdd space wheelC1
    H.spaceAdd space wheelC2
    -- Motor
    motor <- H.newConstraint bodyworkB wheelB (H.SimpleMotor 0)
    H.spaceAdd space motor
    let motorControls = map turn [0, -1, 1]
        turn = H.redefineC motor . H.SimpleMotor . (*4)
    -- Return
    let constrs = [H.forgetC wheelC1, H.forgetC wheelC2, H.forgetC motor]
    return (wheelB, wheelS, constrs, motorControls)
  ---- Removing and drawing
  let drawCar = do
        drawMyShape bodyworkS bodyworkT
        mapM_ (flip drawMyShape wheelT) wheelSs
  let removeCar = do
        mapM_ (H.spaceRemove space) (bodyworkB : wheelBs)
        mapM_ (H.spaceRemove space) (bodyworkS : wheelSs)
        mapM_ (H.spaceRemove space) (concat wheelCs)
  ---- Motor controls
  let control w = mapM_ (!!n) wheelMs where n = fromEnum w
  return ((bodyworkS, (drawCar, removeCar)), control)

-- | Destroy a state.
destroyState :: State -> IO ()
destroyState (State {stSpace = space}) = do
  H.freeSpace space




------------------------------------------------------------
-- Main function and main loop
------------------------------------------------------------

-- | Entry point.
main :: IO ()
main = do
  -- Initialize Chipmunk, GLFW and our state
  H.initChipmunk
  assertTrue initialize "Failed to init GLFW"
  stateVar <- initialState >>= newIORef

  -- Create a window
  assertTrue (openWindow (GL.Size 1200 900) [] Window) "Failed to open a window"
  windowTitle GL.$= "Hipmunk Playground"

  -- Define some GL parameters for the whole program
  clearColor  GL.$= Color4 1 1 1 1
  pointSmooth GL.$= Enabled
  pointSize   GL.$= 3
  lineSmooth  GL.$= Enabled
  lineWidth   GL.$= 2.5
  blend       GL.$= Enabled
  blendFunc   GL.$= (SrcAlpha, OneMinusSrcAlpha)
  matrixMode  GL.$= Projection
  loadIdentity
  ortho (-320) 320 (-240) 240 (-1) 1
  translate (Vector3 0.5 0.5 zero)

  -- Add some callbacks to GLFW
  windowCloseCallback GL.$= exitWith ExitSuccess
  windowSizeCallback  GL.$= (\size -> viewport GL.$= (Position 0 0, size))
  mouseButtonCallback GL.$= processMouseInput stateVar

  -- Let's go!
  now <- GL.get time
  loop stateVar now

-- | The simulation loop.
loop :: IORef State -> Time -> IO ()
loop stateVar oldTime = do
  -- Some key states
  slowKey  <- getKey (SpecialKey ENTER)
  quitKey  <- getKey (SpecialKey ESC)
  clearKey <- getKey (SpecialKey DEL)
  leftKey  <- getKey (SpecialKey LEFT)
  rightKey <- getKey (SpecialKey RIGHT)

  -- Quit?
  when (quitKey == Press) (terminate >> exitWith ExitSuccess)

  -- Clear?
  when (clearKey == Press) $ do
    destroyState =<< readIORef stateVar
    H.resetShapeCounter
    initialState >>= writeIORef stateVar

  -- Update display and time
  updateCar stateVar leftKey rightKey
  updateDisplay stateVar slowKey
  newTime <- advanceTime stateVar oldTime slowKey
  loop stateVar newTime

-- | Updates the car state
updateCar :: IORef State -> KeyButtonState -> KeyButtonState -> IO ()
updateCar stateVar leftKey rightKey = do
  let wantsToBe = case (leftKey, rightKey) of
                    (Press, _) -> GoingLeft
                    (_, Press) -> GoingRight
                    _          -> Stopped
  state <- readIORef stateVar
  when (stCarState state /= wantsToBe) $ do
    stControls state wantsToBe
    writeIORef stateVar (state {stCarState = wantsToBe})

-- | Advances the time.
advanceTime :: IORef State -> Time -> KeyButtonState -> IO Time
advanceTime stateVar oldTime slowKey = do
  newTime <- GL.get time

  -- Advance simulation
  let slower = if slowKey == Press then slowdown else 1
      mult   = frameSteps / (framePeriod * slower)
      framesPassed   = truncate $ mult * (newTime - oldTime)
      simulNewTime   = oldTime + toEnum framesPassed / mult
  advanceSimulTime stateVar $ min maxSteps framesPassed

  -- Correlate with reality
  newTime' <- GL.get time
  let diff = newTime' - simulNewTime
      sleepTime = ((framePeriod * slower) - diff) / slower
  when (sleepTime > 0) $ sleep sleepTime
  return simulNewTime






------------------------------------------------------------
-- Display related functions
------------------------------------------------------------


-- | Renders the current state.
updateDisplay :: IORef State -> KeyButtonState -> IO ()
updateDisplay stateVar slowKey = do
  state <- SV.get stateVar
  clear [ColorBuffer]
  drawInstructions
  when (slowKey == Press) drawSlowMotion
  forM_ (M.assocs $ stShapes state) (fst . snd) -- Draw each one
  readIORef (stCollisionsVar state) >>= mapM_ (drawPoint CollisionPoint)
  swapBuffers

drawInstructions :: IO ()
drawInstructions = preservingMatrix $ do
  translate (Vector3 (-320) 240 zero)
  scale 0.75 0.75 (1 `asTypeOf` zero)
  let render str = do
        translate (Vector3 zero (-16) zero)
        renderString Fixed8x16 str

  color $ Color3 zero zero 1
  render "Press the left mouse button to create a ball."
  render "Press the right mouse button to create a square."
  render "Press the middle mouse button to create a triangle on a pendulum."

  color $ Color3 1 zero zero
  render "Hold LEFT SHIFT to create counterclockwise rotating objects."
  render "Hold RIGHT SHIFT to create clockwise rotating objects."
  render "Hold LEFT or RIGHT to move the car."
  render "Hold ENTER to see in slow motion."

  color $ Color3 zero zero zero
  render "Press DEL to clear the screen."

drawSlowMotion :: IO ()
drawSlowMotion = preservingMatrix $ do
  scale 2 2 (1 `asTypeOf` zero)
  translate (Vector3 (-40) zero zero)
  color $ Color3 zero 1 zero
  renderString Fixed8x16 "Slowwww..."

-- | Draws a shape (assuming zero offset)
drawMyShape :: H.Shape -> H.ShapeType -> IO ()
drawMyShape shape (H.Circle radius) = do
  H.Vector px py <- SV.get $ H.position $ H.body shape
  angle          <- SV.get $ H.angle    $ H.body shape

  color $ Color3 zero zero zero
  renderPrimitive LineStrip $ do
    let segs = 20; coef = 2*pi/toEnum segs
    forM_ [0..segs] $ \i -> do
      let r = toEnum i * coef
          x = radius * cos (r + angle) + px
          y = radius * sin (r + angle) + py
      vertex' x y
    vertex' px py
  drawPoint PositionPoint (px +: py)
drawMyShape shape (H.LineSegment p1 p2 _) = do
  let v (H.Vector x y) = vertex' x y
  pos <- SV.get $ H.position $ H.body shape
  color $ Color3 zero zero zero
  renderPrimitive Lines $ v (p1 + pos) >> v (p2 + pos)
  drawPoint PositionPoint pos
drawMyShape shape (H.Polygon verts) = do
  pos   <- SV.get $ H.position $ H.body shape
  angle <- SV.get $ H.angle    $ H.body shape
  let rot = H.rotate $ H.fromAngle angle
      verts' = map ((+pos) . rot) verts
  color $ Color3 zero zero zero
  renderPrimitive LineStrip $ do
    forM_ (verts' ++ [head verts']) $ \(H.Vector x y) -> do
      vertex' x y
  drawPoint PositionPoint pos

-- | Draws a red point.
drawPoint :: PointType -> H.Vector -> IO ()
drawPoint pt (H.Vector px py) = do
  color $ case pt of
            PositionPoint  -> Color3 zero zero 1
            CollisionPoint -> Color3 1 zero zero
  renderPrimitive Points $ do
    vertex' px py

data PointType = PositionPoint | CollisionPoint
                 deriving (Eq, Ord, Show, Enum)






------------------------------------------------------------
-- Input processing
------------------------------------------------------------

-- | Returns the current mouse position in our space's coordinates.
getMousePos :: IO H.Position
getMousePos = do
  Position cx cy <- GL.get mousePos
  Size _ h <- GL.get $ windowSize
  model    <- GL.get $ matrix (Just $ Modelview 0)
  proj     <- GL.get $ matrix (Just Projection)
  view     <- GL.get $ viewport
  let src = Vertex3 (fromIntegral cx) (fromIntegral h - fromIntegral cy) 0
  Vertex3 mx my _ <- unProject src (model :: GLmatrix GLdouble) proj view
  return (realToFrac mx +: realToFrac my)

-- | Process a user mouse button press.
processMouseInput :: IORef State -> MouseButton -> KeyButtonState -> IO ()
processMouseInput _        _   Press   = return ()
processMouseInput stateVar btn Release = do
  rotateKeyCCW <- getKey (SpecialKey LSHIFT)
  rotateKeyCW  <- getKey (SpecialKey RSHIFT)
  let angVel = case (rotateKeyCCW, rotateKeyCW) of
                 (Press,   Release) -> 50
                 (Release, Press)   -> (-50)
                 _                  -> 0
  (shape,add,draw,remove) <- (case btn of
    ButtonLeft  -> createCircle
    ButtonRight -> createSquare
    _           -> createTriPendulum) angVel

  state <- SV.get stateVar
  let space = stSpace state
  add space >> stateVar SV.$= state {
    stShapes = M.insert shape (draw, remove space) $ stShapes state}





------------------------------------------------------------
-- Object creation
------------------------------------------------------------


-- | The return of functions that create objects.
type Creation = (H.Shape,          -- ^ A representative shape
                 H.Space -> IO (), -- ^ Function that add the entities
                 IO (),            -- ^ Function that draws the entity
                 H.Space -> IO ()  -- ^ Function that removes the entities
                )

-- | The type of the functions that create objects.
type Creator = H.CpFloat -> IO Creation

createCircle :: Creator
createCircle angVel = do
  let mass   = 20
      radius = 20
      t = H.Circle radius
  b <- H.newBody mass $ H.momentForCircle mass (0, radius) 0
  s <- H.newShape b t 0
  (H.position b SV.$=) =<< getMousePos
  H.angVel     b SV.$= angVel
  H.friction   s SV.$= 0.5
  H.elasticity s SV.$= 0.9
  let add space = do
        H.spaceAdd space b
        H.spaceAdd space s
  let draw = do
        drawMyShape s t
  let remove space = do
        H.spaceRemove space b
        H.spaceRemove space s
  return (s,add,draw,remove)

createSquare :: Creator
createSquare angVel = do
  let mass  = 18
      verts = [-15 +: -15, -15 +: 15, 15 +: 15, 15 +: -15]
      t = H.Polygon verts
  b <- H.newBody mass $ H.momentForPoly mass verts 0
  s <- H.newShape b t 0
  (H.position b SV.$=) =<< getMousePos
  H.angVel     b SV.$= angVel
  H.friction   s SV.$= 0.5
  H.elasticity s SV.$= 0.6
  let add space = do
        H.spaceAdd space b
        H.spaceAdd space s
  let draw = do
        drawMyShape s t
  let remove space = do
        H.spaceRemove space b
        H.spaceRemove space s
  return (s,add,draw,remove)

createTriPendulum :: Creator
createTriPendulum angVel = do
  let mass  = 100
      verts = [-30 +: -30, 0 +: 37, 30 +: -30]
      t = H.Polygon verts
  b <- H.newBody mass $ H.momentForPoly mass verts 0
  s <- H.newShape b t 0
  (H.position b SV.$=) =<< getMousePos
  H.angVel     b SV.$= angVel
  H.friction   s SV.$= 0.8
  H.elasticity s SV.$= 0.3

  let staticPos = 0 +: 240
  static <- H.newBody H.infinity H.infinity
  H.position static SV.$= staticPos
  j <- H.newConstraint static b (H.Pin 0 0)

  let add space = do
        H.spaceAdd space b
        H.spaceAdd space s
        H.spaceAdd space j
  let remove space = do
        H.spaceRemove space b
        H.spaceRemove space s
        H.spaceRemove space j
  let draw = do
        H.Vector x1 y1 <- SV.get $ H.position b
        let H.Vector x2 y2 = staticPos
        color $ Color3 (0.7 `asTypeOf` zero) 0.7 0.7
        renderPrimitive LineStrip $ do
          let
          vertex' x1 y1
          vertex' x2 y2
        drawMyShape s t
  return (s,add,draw,remove)

vertex' :: Double -> Double -> IO ()
vertex' x y = let f p = realToFrac p :: GLdouble
              in vertex (Vertex2 (f x) (f y))






------------------------------------------------------------
-- Simulation bookkeeping
------------------------------------------------------------

-- | Advances the time in a certain number of steps.
advanceSimulTime :: IORef State -> Int -> IO ()
advanceSimulTime _        0     = return ()
advanceSimulTime stateVar steps = do
  removeOutOfSight stateVar
  state <- SV.get stateVar

  -- Do (steps-1) steps that clear the collisions variable.
  let clearCollisions = writeIORef (stCollisionsVar state) []
      step = H.step (stSpace state) frameDelta
  replicateM_ (steps-1) $ step >> clearCollisions

  -- Do a final step that will leave the collisions variable filled.
  step

-- | Removes all shapes that may be out of sight forever.
removeOutOfSight :: IORef State -> IO ()
removeOutOfSight stateVar = do
  state   <- SV.get stateVar
  shapes' <- foldM f (stShapes state) $ M.assocs (stShapes state)
  stateVar SV.$= state {stShapes = shapes'}
    where
      f shapes (shape, (_,remove)) = do
        H.Vector x y <- SV.get $ H.position $ H.body shape
        if y < (-350) || abs x > 800
          then remove >> return (M.delete shape shapes)
          else return shapes
