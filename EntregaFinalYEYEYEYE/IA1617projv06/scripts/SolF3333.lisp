(load "datastructures.lisp")
(load "auxfuncs.lisp")


;;; TAI position
(defun make-pos (c l)
  (list c l))
(defun pos-l (pos)
  (first pos))
(defun pos-c (pos)
  (second pos))

;;; TAI acceleration
(defun make-acce (c l)
  (list c l))
(defun acce-l (pos)
  (first pos))
(defun acce-c (pos)
  (second pos))

;;; TAI velocity
(defun make-vel (c l)
  (list c l))
(defun vel-l (pos)
  (first pos))
(defun vel-c (pos)
  (second pos))


;; Solution of phase 1

(defun onTrack (pos track)
	  (not (or (< (pos-l pos) 0) (< (pos-c pos) 0)
      	(>= (pos-l pos) (pos-l (track-size track)))
      	(>= (pos-c pos) (pos-c (track-size track)))
      ))
)


(defun getTrackContent (pos track)
  (nth (pos-c pos) (nth (pos-l pos) (track-env track))))

;; Pedir 0,4
(defun isObstaclep (pos track)
  "check if the position pos is an obstacle"
  (or (< (pos-l pos) 0) (< (pos-c pos) 0)
      (>= (pos-l pos) (pos-l (track-size track)))
      (>= (pos-c pos) (pos-c (track-size track)))
      (null (getTrackContent pos track))))

;; Pedir 0,4
(defun isGoalp (st) 
  "check if st is a solution of the problem"
  (let ((current-position (state-pos st))
	(track (state-track st)))
    (and (member current-position (track-endpositions track) :test #'equalp)
	 T)))

;; Pedir 1,2
(defun nextState (st act)
  "generate the nextState after state st and action act from prolem"
  (let ((new-state (make-state :action act :track (state-track st))))
    (setf (state-vel new-state)
	  (make-vel (+ (vel-l (state-vel st)) (acce-l act))
		    (+ (vel-c (state-vel st)) (acce-c act))))
    (setf (state-pos new-state)
	  (make-pos (+ (pos-l (state-pos st)) (vel-l (state-vel new-state)))
		    (+ (pos-c (state-pos st)) (vel-c (state-vel new-state)))))
    (setf (state-cost new-state)
	  (cond ((isGoalp new-state) -100)
		((isObstaclep (state-pos new-state) (state-track new-state)) 20)
		(T 1)))
    (when (= (state-cost new-state) 20)
      (setf (state-vel new-state) (make-vel 0 0))
      (setf (state-pos new-state) (make-pos (pos-l (state-pos st))
					    (pos-c (state-pos st)))))
    (values new-state)))

;; Solution of phase 2

;;; Pedir 
(defun nextStates (st)
  "generate all possible next states"
  (let ((successors nil))
    (dolist (act (possible-actions) successors)
      (let ((new-state (nextState st act)))
	(if (not (member new-state successors :test #'equalp))
	    (push new-state successors))))))

;;; Solucao e uma seq ordenada de estados
(defun solution (node)
  (let ((seq-states nil))
    (loop 
      (when (null node)
	(return)) 
      (push (node-state node) seq-states)
      (setf node (node-parent node)))
    (values seq-states)))


;;; limdepthfirstsearch 
(defun limdepthfirstsearch (problem lim &key cutoff?)
  "limited depth first search
     st - initial state
     problem - problem information
     lim - depth limit"
  (labels ((limdepthfirstsearch-aux (node problem lim)
	     (if (isGoalp (node-state node))
		 (solution node)
		 (if (zerop lim)
		     :cutoff
		     (let ((cutoff? nil))
		       (dolist (new-state (nextStates (node-state node)))
			 (let* ((new-node (make-node :parent node :state new-state))
				(res (limdepthfirstsearch-aux new-node problem (1- lim))))
			   (if (eq res :cutoff)
			       (setf cutoff? :cutoff)
			       (if (not (null res))
				   (return-from limdepthfirstsearch-aux res)))))
		       (values cutoff?))))))
    (let ((res (limdepthfirstsearch-aux (make-node :parent nil :state (problem-initial-state problem))
					problem
					lim)))
      (if (eq res :cutoff)
	  (if cutoff?
	      :cutoff
	      nil)
	  res))))
				      

;iterlimdepthfirstsearch
(defun iterlimdepthfirstsearch (problem &key (lim most-positive-fixnum))
  "limited depth first search
     st - initial state
     problem - problem information
     lim - limit of depth iterations"
  (let ((i 0))
    (loop
      (let ((res (limdepthfirstsearch problem i :cutoff? T)))
	(when (and res (not (eq res :cutoff)))
	  (return res))
	(incf i)
	(if (> i lim)
	    (return nil))))))


	
;; Solution of phase 3

;; Heuristic
 
; (defun compute-distance (pos1 pos2)
;   (let ((dx (- (pos-c pos1) (pos-c pos2)))
;        (dy (- (pos-l pos1) (pos-l pos2))))
;        (isqrt (+ (* dx dx) (* dy dy)))
;   )
; )

; (defun compute-heuristic (st)
;   (let ((goals (track-endpositions (state-track st)))
;         (track (state-track st))
;         (pos (state-pos st))
;         (h most-positive-fixnum))
;     (when (not (isObstaclep pos track)) 
;         (dolist (goal goals)
;           (setf h (min h (compute-distance pos goal)))
;       )
;     )
;     h
;   )
;  )

(defun compute-heuristic2 (st)
	(when (isObstaclep (state-pos st) (state-track st)) (return-from compute-heuristic2 most-positive-fixnum))
	(let ((dist most-positive-fixnum) 
		(x 0)
		(y 0)
		(distX 0)
		(distY 0)
		(speedX (vel-c (state-vel st)))
		(speedY (vel-l (state-vel st))))
        (when (null speedX) (setf speedX 0))
        (when (null speedY) (setf speedY 0))
		(dolist (endpos (track-endpositions (state-track st)))
			(setf x (- (pos-c endpos) (pos-c (state-pos st))))
			(setf y (- (pos-l endpos) (pos-l (state-pos st))))
			(when (< (+ (abs x) (abs y)) dist)
				(setf distX x)
				(setf distY y)
				(setf dist (+ (abs x) (abs y)))
			)
		)
        (setf x 0)
        (setf y 0)
		(when (< distX 0)
			(setf distX  (* distX -1))
			(setf speedX (* speedX -1))
		)
		(when (< distY 0)
			(setf distY  (* distY -1))
			(setf speedY (* speedY 0))
		)
		(loop while (> distX 0) do
			(incf speedX)
            (setf distX (- distX speedX))
            (incf x)
		)
		(loop while (> distY 0) do
			(incf speedY)
            (setf distY (- distY speedY))
            (incf y)
		)
        (max x y)
	)
)

(defun compute-heuristic (st)
	(when (isObstaclep (state-pos st) (state-track st)) (return-from compute-heuristic most-positive-fixnum))
	(let ((h most-positive-fixnum) 
		(x 0)
		(y 0))
		(dolist (endpos (track-endpositions (state-track st)))
			(setf x (abs (- (pos-c endpos) (pos-c (state-pos st)))))
			(setf y (abs (- (pos-l endpos) (pos-l (state-pos st)))))
			(setf h (min h (max x y)))
		)
		h
	)
)
	    
;;; A*
(defun insertOpen (node openNodes)
  (cond
    ((null openNodes) (list node))
    ((<= (node-f node) (node-f (first openNodes))) (cons node openNodes))
    (t (cons (first openNodes) (insertOpen node (rest openNodes))))
  )
)

(defun memberClosed (state lst)
    (member state lst :test (lambda (x y) (and (equalp (state-vel x) (state-vel y)) (equalp (state-pos x) (state-pos y)))))
)

(defun memberOpen (node lst)
    (first (member node lst :test (lambda (x y) (and (equalp (state-vel (node-state x)) (state-vel (node-state y))) (equalp (state-pos (node-state x)) (state-pos (node-state y)))))))
)

(defun a* (problem)
  (let ((initialNode (make-node :parent nil :state (problem-initial-state problem)
                         :g 0 
                         :h (funcall (problem-fn-h problem) (problem-initial-state problem))))
        (closedStates ())
        (openNodes ())
       )
    (setf (node-f initialNode) (node-h initialNode))
    (push initialNode openNodes)
    (loop while openNodes do
       (let ((node (pop openNodes)))
         (when (funcall (problem-fn-isGoal problem) (node-state node)) (return-from a* (solution node)))
         (push (node-state node) closedStates)
         (loop for child in (funcall (problem-fn-nextStates problem) (node-state node)) do
             (block innerLoop
               (when (isObstaclep (state-pos child) (state-track child)) (return-from innerLoop))
               (let ((childNode (make-node :parent node :state child
                                            :g (+ (state-cost child) (node-g node))
                                            :h (funcall (problem-fn-h problem) child)))
                     (nodeInOpen nil))
                 (setf (node-f childNode) (+ (node-g childNode) (node-h childNode)))
                 (when (memberClosed (node-state childNode) closedStates) (return-from innerLoop))
                 (setf nodeInOpen (memberOpen childNode openNodes))
                 (if nodeInOpen 
                   (when (> (node-g nodeInOpen) (node-g childNode))
                        (setf (node-state nodeInOpen) (node-state childNode))
                        (setf (node-g nodeInOpen) (node-g childNode))
                        (setf (node-parent nodeInOpen) node)
                        (setf (node-f nodeInOpen) (node-f childNode))
                   )
                   (setf openNodes (insertOpen childNode openNodes))
                 )
               )
             )
         )
       )
    )
    nil
  )
)
;;Best search first auxiliar functions -----------

(defun normalize (pos1 pos2)

  (let* ((direction (list (- (pos-l pos2) (pos-l pos1)) (- (pos-c pos2) (pos-c pos1))))
       (norma (sqrt (+ (expt (first direction) 2 ) (expt (second direction) 2 )))))

    (setf (first direction) (/ (first direction) norma))
    (setf (second direction) (/ (second direction) norma))
    direction
  )
)

(defun sumDirection (pos1 direction)
  (list (+ (first pos1) (first direction)) (+ (second pos1) (second direction)))
)


(defun calculatePerpDirection (direction)
  (let ((newDirection (list 0 0)))
    (setf (first newDirection) (second direction))
    (setf (second newDirection) (- 0 (first direction)))
    newDirection
  )
)

(defun roundPoint (point)
 (list (round (first point)) (round (second point)))
)

(defun calculateClosestPoint (currentPoint direction track)
  (if (isObstaclep (roundPoint currentPoint) track)
  (let ((rightPoint currentPoint) (leftPoint currentPoint)
     (distance 0)
     (distance2 0))

    ;Find the closest point to the "left"
    (loop while (and (onTrack (roundPoint leftPoint) track) (isObstaclep (roundPoint leftPoint) track)) do
      (setf leftPoint (sumDirection leftPoint direction))
      (setf distance (1+ distance))
    )

    ;Find the closest point to the "right"
    (setf direction (list (- (first direction)) (- (second direction))))

    (loop while (and (onTrack rightPoint track) (isObstaclep (roundPoint rightPoint) track)) do
      (setf rightPoint (sumDirection rightPoint direction))
      (setf distance2 (1+ distance2))
    )


    (setf leftPoint (if (and (not (isObstaclep (roundPoint leftPoint) track)) (or (< distance distance2) (isObstaclep (roundPoint rightPoint) track))) leftPoint rightPoint))

    (if (isObstaclep (roundPoint leftPoint) track) currentPoint leftPoint)

  ) currentPoint)
)

(defun masterLineList (problem)
  (let* ( (masterLine (list (state-pos (problem-initial-state problem)))) 
       (finalPos (car (track-endpositions (state-track (problem-initial-state problem))))) 
       (direction (normalize (car masterLine) finalPos))
       (currentPoint (car masterLine))
       (closestPoint nil)
       (perpendicularDir (calculatePerpDirection direction))
       (tmpPos nil)
      )
    (loop while (not (equal (roundPoint currentPoint) finalPos)) do

      (setf currentPoint (sumDirection currentPoint direction))

      (setf closestPoint (calculateClosestPoint currentPoint perpendicularDir (state-track (problem-initial-state problem))))

      (if (and    (not (NULL tmpPos) ) 
            (not (eq (round (first closestPoint)) (round (first tmpPos))))
            (not (eq (round (second closestPoint)) (round (second tmpPos))))
        )
        (progn (setf masterLine (append masterLine (list (roundPoint tmpPos) ))))
      )
      (setf tmpPos closestPoint)

    )
  (print "MASTER LINE")
  (print masterLine)
  
  (rest masterLine)
  )
)
;;FIX ME MASTRELINE STRUCTURE

(defun best-search (problem)

  (let ((masterLine (masterLineList problem)) 
     (originalGoal (track-endpositions (state-track (problem-initial-state problem))))
     (initialSt (problem-initial-state problem))
     (resultPath (list (problem-initial-state problem)))
     (resultAux nil))
    
    (loop for shortcut in masterLine do
      (print "Doing another a*")
      (setf (state-cost initialSt) 1)
      (setf (problem-initial-state problem) initialSt)
      (setf (track-endpositions (state-track (problem-initial-state problem))) (list shortcut))
      (setf resultAux (a* problem))
      (if (not(NULL resultAux)) (setf resultPath(append resultPath (rest resultAux))))
      (setf initialSt (car (last resultAux)))
    )
    (setf (state-cost (car (last resultAux))) 1)
    (setf (problem-initial-state problem) (car (last resultAux)))
    (setf (track-endpositions (state-track (problem-initial-state problem))) originalGoal)
    (setf resultAux (a* problem))
    (setf resultPath(append resultPath (rest resultAux)))
  (print "Final result")
  (print (format nil "~{~a~^~}" (states-to-list resultPath)))
  resultPath
  )
)
