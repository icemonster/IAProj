;81423 Joana Hrotko 81714 Nuno Sabino Grupo: 27
;;; These functions, and any other ones needed must be implemented

(load "datastructures.lisp")
(load "auxfuncs.lisp")


(defun isObstaclep (pos track) 
  "check if there is an obstacle at position pos of the track"
  (setf xIndex (nth 0 pos))
  (setf yIndex (nth 1 pos))

  (eq nil (nth yIndex (nth xIndex (track-env track)))) 

)

(defun isGoalp (st) 
  "check if st is a goal state"
  ;member primitive will return a list if succeeded or nil if it
  ;didnt find the position in the track. 
  (not (null (member (state-pos st) (track-endpositions (state-track st)) :test 'equal)))
)

(defun nextState (st act)
  "generate the nextState after state st and action act"
  (setf newVelX (+ (car act) (car (state-vel st))))
  (setf newVelY (+ (second act) (second (state-vel st))))

  (setf posX (+ (car (state-pos st)) newVelX))
  (setf posY (+ (second (state-pos st)) newVelY))
	
  (setf next (make-STATE :POS (list posX posY)
		  :VEL (list newVelX newVelY)
		  :ACTION act
		  :COST 1
		  :TRACK (state-track st)
		  :OTHER NIL))

  (cond
  	((isGoalp next) (setf (state-cost next) -100))
  	((isObstaclep (list posX posY) (state-track next)) (setf (state-cost next) 20))
  )
  
  next
)