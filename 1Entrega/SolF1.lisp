
;;; These functions, and any other ones needed must be implemented

;;; Utilizar estes includes para os testes na versao local
;;; comentar antes de submeter
(load "datastructures.lisp")
(load "auxfuncs.lisp")

;;; Utilizar estes includes para a versao a submeter
; tirar o comentario antes de submeter
;(load "datastructures.fas")
;(load "auxfuncs.fas")

(defun isObstaclep (pos track) 
  "check if there is an obstacle at position pos of the track"
  (let ((xIndex (nth 0 pos)) (yIndex (nth 1 pos)))

  (eq nil (nth yIndex (nth xIndex (track-env track)))))

)

(defun isGoalp (st) 
  "check if st is a goal state"
  ;member primitive will return a list if succeeded or nil if it
  ;didnt find the position in the track. 
  (not (null (member (state-pos st) (track-endpositions (state-track st)) :test 'equal)))
)

(defun nextState (st act)
  "generate the nextState after state st and action act"
  (let ((newVelX nil) (newVely nil) (posX nil) (posY nil) (next nil))
  (setq newVelX (+ (car act) (car (state-vel st))))
  (setq newVelY (+ (second act) (second (state-vel st))))

  (setq posX (+ (car (state-pos st)) newVelX))
  (setq posY (+ (second (state-pos st)) newVelY))
	
  (setq next (make-STATE :POS (list posX posY)
		  :VEL (list newVelX newVelY)
		  :ACTION act
		  :COST 1
		  :TRACK (state-track st)
		  :OTHER NIL))

  (cond
  	((isGoalp next) (setf (state-cost next) -100))
  	((isObstaclep (list posX posY) (state-track next)) (setf (state-cost next) 20))
  )
  
  next)
)