
## Projet Inverse Kinematic 
 --- 
* Auteur :  Bryan Bachelet
* Unity Version : 2022.3.10f
* Instructions : La scène finale est IK Result. Les scripts importants sont :
	* IK_Jaccobian : 
		* Le script principale où se trouve la logique générale du solver. Le solver réalisé est le Jaccobian Solver. Le script se trouve sur la jambe gauche du personnage 
		
		* La résolution de la position et les contrainte sont fonctionnelles. 
		
		* Tentative de mettre en place la résolution de la position et la rotation mais sans succès pour activer la fonctionnalité, cocher le boolean "Is Generic Jaccobian Active". L'ensemble des tentatives ont été garder dans le code comme la mise en place d'un de algorithme SVD pour l'inversion de la matrice Jaccobian afin d'avoir un solver plus robuste. Consultation de l'aide Shahin (Amir H.) Rabbani avec  son article et des échange de mail:  https://www.shahinrabbani.ca/jacobian/a-recipe-to-cook-jacobian
	
	* Joint Contraint :
		* Contient la logique pour contraindre les joints et leurs rotations. Le script à dût être adapté pour le modèle de personnage et ses contraintes
		
	* Simple Jacobian Matrix & Generic Jacobian :
		* Script qui contient les classes personnalisé pour le Jacobian et ses opérations pour le solver
	* Le reste des script sont soit pour habiller la scène ou des tests qui non pas de pertinence. 
