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


(defstruct pontoAux pos dist)

;; Solution of phase 1

(defun getTrackContent (pos track)
  (nth (pos-c pos) (nth (pos-l pos) (track-env track))))

;; Pedir 0,4

(defun onTrack (pos track)
	  (not (or (< (pos-l pos) 0) (< (pos-c pos) 0)
      	(>= (pos-l pos) (pos-l (track-size track)))
      	(>= (pos-c pos) (pos-c (track-size track)))
      ))
)

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
  (let ((new-state (make-state :action act :track (state-track st) :other 0)))
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
  "limited depth first search"
     st - initial state
     problem - problem information
     lim - depth limit
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
  "limited depth first search"
     st - initial state
     problem - problem information
     lim - limit of depth iterations
  (let ((i 0))
    (loop
      (let ((res (limdepthfirstsearch problem i :cutoff? T)))
	(when (and res (not (eq res :cutoff)))
	  (return res))
	(incf i)
	(if (> i lim)
	    (return nil))))))

	
;; Solution of phase 3


(defun distance (pos1 pos2)
  (max (abs (- (pos-l pos2) (pos-l pos1))) (abs (- (pos-c pos2) (pos-c pos1))))
  ;;(isqrt (+ (expt (- (pos-l pos1) (pos-l pos2)) 2 ) (expt (- (pos-c pos1) (pos-c pos2)) 2 )))
)
;; Heuristic
(defun compute-heuristic (st)
  (let ((position (state-pos st)) (minDistance most-positive-fixnum) (heuristicDistance 0)) 
    (cond 
      ((isGoalp st) 0)
      ((isObstaclep position (state-track st)) most-positive-fixnum)
      (t 
        (loop for endposition in (track-endpositions (state-track st))
          do (setf heuristicDistance (distance position endposition))
             (if (< heuristicDistance minDistance) (setf minDistance heuristicDistance) nil)
        )
      )
    )
    heuristicDistance )
)

(defun nodeEqual(node1 node2)
	(and (equalp (state-pos (node-state node1)) (state-pos (node-state node2))) (eq (state-vel (node-state node1)) (state-vel (node-state node2))))
)

(defun inList(node lst)
	(loop for elem in lst do
		(if (nodeEqual node elem) (return-from inList elem))
	)
	0
)

(defun insertLst (node lst)
  "Inserts NODE onto LST, according to FVALUE ordering."
  (cond ((null lst)(list node))
        ((<= (node-f node)
            (node-f (first lst)) )
         (cons node lst) )
        (t (cons (first lst)
                 (insertLst node (rest lst)) )) ) 
)

(defun computeBestHeuristic (st masterLine)
	(if (NULL (state-other st)) (setf (state-other st) 0))
	(if (eq (state-other st) (length masterLine)) (compute-heuristic st) 
							(progn
	 							(if (eq (state-pos st) (pontoAux-pos (nth (state-other st) masterLine)))
	 													(setf (state-other st) (1+ (state-other st)))
	 							) 
								(if (eq (state-other st) (length masterLine)) (compute-heuristic st) (+ (pontoAux-dist (nth (state-other st) masterLine)) (distance (state-pos st) (pontoAux-pos (nth (state-other st) masterLine)))))
							) 
	)
)

;;; A* 
(defun a* (problem &optional (masterLine NIL masterLine-supplied-p) );;masterLina lista de posicoes auxiliares
	(let ((openSet (list (make-node :state (problem-initial-state problem)
                            :parent nil 
                            :g 0
                            :h (if masterLine-supplied-p (funcall (problem-fn-h problem) (problem-initial-state problem) masterLine) 
									     				 (funcall (problem-fn-h problem) (problem-initial-state problem)))
                            :f (if masterLine-supplied-p (funcall (problem-fn-h problem) (problem-initial-state problem) masterLine) 
									     				 (funcall (problem-fn-h problem) (problem-initial-state problem))))))
        (closedSet nil)
        (current nil)
        (tempNode nil)
        (tentativeG 0))
    (loop while (not (NULL openSet)) do
      (setf current (car openSet))

      ;; take the lowest f from openSet
      (if (funcall (problem-fn-isGoal problem) (node-state current)) (return-from a* (reconstructPath current problem)))
      (setf openSet (rest openSet))
      (insertLst current closedSet)
      (loop for neighbor in (funcall (problem-fn-nextStates problem) (node-state current)) do
      	(let ((neighborNode (make-node :state neighbor)))
      		(setf tentativeG (+ (state-cost neighbor) (node-g current)))
      		(setf tempNode (inList neighborNode openSet)) 
      		(if (eq 0 tempNode) (progn 	 (setf (node-parent neighborNode) current)
									     (setf (node-g neighborNode) (+ (state-cost (node-state neighborNode)) (node-g current)))
									     (setf (node-h neighborNode) (if masterLine-supplied-p (funcall (problem-fn-h problem) (node-state neighborNode) masterLine) 
									     													   (funcall (problem-fn-h problem) (node-state neighborNode))))
									     (setf (node-f neighborNode) (+ (node-g neighborNode) (node-h neighborNode)))
      									 (setf openSet (insertLst neighborNode openSet)))
      							(progn (setf neighborNode tempNode) (if (< tentativeG (node-g neighborNode)) 
      								(progn 
      									(setf (node-parent neighborNode) current)
      									(setf (node-g neighborNode) tentativeG)
      									(setf (node-f neighborNode) (+ tentativeG (node-h neighborNode)))
      									))))

      	)
      )
    )
  )
  nil
)

(defun reconstructPath (current problem)
  (let ((path (list (node-state current))))
  	(loop while (not (eq (node-state current) (problem-initial-state problem))) do
  		(setf current (node-parent current))
  		(setf path (cons (node-state current) path))
  	)
  	(print (format nil "~{~a~^~}" (states-to-list path)))	
  	path
  )
)


;;Best search first auxiliar functions -----------

(defun normalize (pos1 pos2)
	(print "AHAHAOOOH")
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

		(print "LOOK AT THESE VALUES")
		(print leftPoint)
		(print rightPoint)
		(setf leftPoint (if (and (not (isObstaclep (roundPoint leftPoint) track)) (or (< distance distance2) (isObstaclep (roundPoint rightPoint) track))) leftPoint rightPoint))
		(print leftPoint)
		(if (isObstaclep (roundPoint leftPoint) track) currentPoint leftPoint)

	) currentPoint)
)

(defun masterLineList (problem)
	(let* ( (masterLine (list (make-pontoAux :pos (state-pos (problem-initial-state problem)) :dist 0))) 
		   (finalPos (car (track-endpositions (state-track (problem-initial-state problem))))) 
		   (direction (normalize (pontoAux-pos (car masterLine)) finalPos))
		   (currentPoint (pontoAux-pos (car masterLine)))
		   (closestPoint nil)
		   (perpendicularDir (calculatePerpDirection direction))
		   (tmpPos nil)
		  )
		(loop while (not (equal (roundPoint currentPoint) finalPos)) do
			(print "--------------------")
			
			(setf currentPoint (sumDirection currentPoint direction))
			(print "CURRENT POINT")
			(print currentPoint)
			(setf closestPoint (calculateClosestPoint currentPoint perpendicularDir (state-track (problem-initial-state problem))))
			(print "Closest point:")
			(print closestPoint)

			(if (and    (not (NULL tmpPos) ) 
						(not (eq (round (first closestPoint)) (round (first tmpPos))))
						(not (eq (round (second closestPoint)) (round (second tmpPos))))
				)
				(progn (setf masterLine (append masterLine (list (make-pontoAux :pos (roundPoint tmpPos) :dist 0)))))
			)
			(setf tmpPos closestPoint)

		)
		
		(let ((indice (- (length masterLine) 2)))
			(setf (pontoAux-dist (car (last masterLine))) (distance (pontoAux-pos (car (last masterLine))) finalPos))
			(loop while (>= indice 0) do 
				(setf (pontoAux-dist (nth indice masterLine)) 
							(+ (distance (pontoAux-pos (nth indice masterLine)) (pontoAux-pos (nth (1+ indice) masterLine))) 
							   (pontoAux-dist (nth (1+ indice) masterLine))))
				(setf indice (1- indice))
			)
		)
	(print "MASTER LINE")
	(print masterLine)
	
	masterLine ;fixme 
	)
)


(defun best-search (problem)
	(setf (problem-fn-h problem) #'computeBestHeuristic)
	(let ((masterLine (masterLineList problem)))
		(print "Oki gonna solve the problem now")
		(a* problem masterLine)
	)
)
