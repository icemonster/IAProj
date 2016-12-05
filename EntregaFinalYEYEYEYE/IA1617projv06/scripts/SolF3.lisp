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
  (let ((possibleStates (list)))
    (loop for action in (possible-actions)
      do (setf possibleStates (cons (nextState st action) possibleStates))
    )
    possibleStates
  )
)

;;; limdepthfirstsearch 
(defun limdepthfirstsearch (problem lim)
  "limited depth first search"
     st - initial state
     problem - problem information
     lim - depth limit
	(list (make-node :state (problem-initial-state problem))) )
				      

;iterlimdepthfirstsearch
(defun iterlimdepthfirstsearch (problem)
  "limited depth first search"
     st - initial state
     problem - problem information
     lim - limit of depth iterations
	(list (make-node :state (problem-initial-state problem))) )
	
;; Solution of phase 3


(defun distance (pos1 pos2)
  (round (sqrt (+ (expt (- (pos-l pos1) (pos-l pos2)) 2 ) (expt (- (pos-c pos1) (pos-c pos2)) 2 ))))
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
    heuristicDistance ))
	  

(defun nodesListAux (st parentNode nodesList)
  (let ((newNode (make-node :state st :parent parentNode )))
    (if (equal parentNode nil) (setf (node-g newNode) 0) 
                               (setf (node-g newNode) (+ (state-cost st) (node-g parentNode))))
    (append nodesList '(newNode))

    (loop for newState in (nextStates st) do 
      (setf nodesList (nodesListAux newState newNode nodesList))
    )
  nodesList 
  )
)

(defun createNodesList (st)
	(let ((nodesList nil))
			(setf nodesList (nodesListAux st nil nodesList))
    nodesList
	)
)

(defun nodeEqual(node1 node2)
	(eq (node-state node1) (node-state node2))
)

(defun inList(node lst)
	(loop for elem in lst do
		(if (nodeEqual node elem) (return-from inList elem))
	)
	0
)

(defun addToList (newNode lst)
	(let ((result nil) (elem nil) (alreadyAdded nil))
	
		(loop while (not (NULL lst)) do
			(setf elem (first lst))
			(if (and (not( alreadyAdded)) (< (node-f newNode) (node-f elem))) 
				(progn (setf alreadyAdded t) 
				(append result '(newNode))))
			(append result '(elem))
			(setf lst (rest lst))
		)
	)
)

(defun addIncompleteNodeList(newNode current lst)
	 (setf (node-parent newNode) current)
     (setf (node-g newNode) (+ (state-cost (node-state newNode)) (node-g current)))
     (setf (node-h newNode) (compute-heuristic newNode)) ;;FIX ALL compute-heuristics (supoosed to be fn-h)
     (setf (node-f newNode) (+ (node-g newNode) (node-h newNode)))
     (addToList newNode lst)
     newNode
)
	    
;;; A* https://en.wikipedia.org/wiki/A*_search_algorithm
(defun a* (problem)

	(let ((openSet (list (make-node :state (problem-initial-state problem)
                            :parent nil 
                            :g 0
                            :h (compute-heuristic (problem-initial-state problem))
                            :f (compute-heuristic (problem-initial-state problem)))))
        (closedSet nil)
        (current nil)
        (tempNode nil)
        (tentativeG 0))
    (loop while (not (NULL openSet)) do
      (setf current (car openSet))
      ;; take the lowest f from openSet
      (if (funcall (problem-fn-isGoal problem) (node-state current)) (return-from a* (reconstructPath current problem)))
      (setf openSet (rest openSet))
      (addToList current closedSet)
      (loop for neighbor in (funcall (problem-fn-nextStates problem) (node-state current)) do	 ;;problem next states
      	(let ((neighborNode (make-node :state neighbor)))
      		(setf tentativeG (+ (state-cost neighbor) (node-g current)))
      		(setf tempNode (inList neighborNode openSet))
      		(if (eq 0 tempNode) (setf neighborNode (addIncompleteNodeList neighborNode current openSet)) 
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
	:failure
)
  
(defun reconstructPath (current problem)
  (let ((path (list (node-state current))))
  	(while (not (eq current (problem-initial-state problem)))
  		(setf current (node-parent current))
  		(cons (node-state current) path)
  	)
  	path
  )
)