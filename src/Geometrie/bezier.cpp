#include <open_rm/Geometrie/bezier.h>
#include <open_rm/Algebre/equations.h>
#include <open_rm/Geometrie/distances.h>

namespace rm{
namespace Geometrie{
namespace Bezier{
/*!
 * \brief Constructeur par défaut
 */
Courbe::Courbe()
{
    _polyOK=false;
    _withPoly=false;
}

/*!
 * \brief Constructeur principal
 * \param ptsControle points de controle de la courbe de Bézier
 * \param withPoly Indique s'il faut précalculer le polynome définissant la courbe et sa dérivée (oui par défaut)
 */
Courbe::Courbe(std::vector<cv::Point2d> ptsControle, bool withPoly)
{
    _pc=ptsControle;
    _polyOK=false;
    _withPoly=withPoly;
    if(withPoly)
        buildPoly();

}

/*!
 * \brief Définit l'ensemble des points de controle de la courbe
 * \param ptsControle Vecteur contenant les nouveaux points de controle
 * \param withPoly Indique s'il faut précalculer le polynome définissant la courbe et sa dérivée (oui par défaut)
 */
void Courbe::set(std::vector<cv::Point2d> ptsControle, bool withPoly)
{
    _pc=ptsControle;
    _polyOK=false;
    _withPoly=withPoly;
    if(withPoly)
        buildPoly();

}

/*!
 * \brief Indique le degré de la courbe portée par la structure
 * \return Degré de la courbe de Bézier
 */
unsigned int Courbe::degre()
{
    return _pc.size()-1;
}

/*!
 * \brief Sépare la courbe en deux courbes vérifiant une connectivité C^1
 *
 * La première des deux courbes écrase la courbe courante stockée par la structure, tandis
 * que la seconde courbe (nouvellement créée) est retournée par la fonction
 *
 * \return la courbe de Bézier correspondant à la seconde portion de la courbe initiale
 */
Courbe Courbe::split()
{
    // La génération de nouveaux points de controle passe par un échantillonnage de la courbe de Bezier
    // (7 points sont nécessaires)
    // On choisit les points p_0,...,p_6 tels que p_i=i/6
    std::vector<cv::Point2d> p;
    for(unsigned int i=0;i<=6;i++){
        p.push_back(computePt((double)i/6.));
    }

    // A partir des coordonées de n points d'une courbe de Bézier de degré n-1 dont on connait la position (valeur de t)
    // il est possible de retrouver les n points de contrôle de la courbe. Pour couper la courbe initiale en deux,
    // on va considérer les points p_0,...,p_3 tels que t=0,1/3,2/3 et 1 pour la première courbe,
    // et p3,...,p6 tels que t=0,1/3,2/3 et 1 pour la seconde courbe.
    // On peut ainsi en déduire les points de contrôle pour chacune des deux courbes
    std::vector<cv::Point2d> pc1,pc2;
    cv::Point2d pt;

    pc1.push_back(p[0]);
    pc1.push_back(1/18.*(-15*p[0]+54*p[1]-27*p[2]+6*p[3]));
    pc1.push_back(1/18.*(6*p[0]-27*p[1]+54*p[2]-15*p[3]));
    pc1.push_back(p[3]);
    pc2.push_back(p[3]);
    pc2.push_back(1/18.*(-15*p[3]+54*p[4]-27*p[5]+6*p[6]));
    pc2.push_back(1/18.*(6*p[3]-27*p[4]+54*p[5]-15*p[6]));
    pc2.push_back(p[6]);

    _pc=pc1;

    return Courbe(pc2);

}

/*!
 * \brief Calcule les coordonnées d'un point appartenant à la courbe
 * \param t position du point considéré (entre 0 et 1)
 * \return coordonnées du point appartenant à la courbe
 */
cv::Point2d Courbe::computePt(long double t)
{
    return deCasteljau(_pc,t);
}

/*!
 * \brief Détermine un point de la courbe en utilisant son expression polynomiale (beaucoup plus rapide)
 * \param t Position du point considéré (entre 0 et 1)
 * \return Coordonnées du point appartenant à la courbe
 */
cv::Point2d Courbe::computePtPoly(double t)
{
    return cv::Point2d(_polyX.compute(t),_polyY.compute(t));
}

/*!
 * \brief Détermine la dérivée d'un point de la courbe en utilisant son expression polynomiale
 * \param t Position du point considéré (entre 0 et 1)
 * \return Vecteur tangent à la courbe en \a t
 */
cv::Point2d Courbe::computePtPrime(double t)
{
    cv::Point2d res;
    res.x=_dpolyX.compute(t);
    res.y=_dpolyY.compute(t);
    return res;
}

/*!
 * \brief Détermine la dérivée seconde d'un point de la courbe en utilisant son expression polynomiale
 * \param t Position du point considéré (entre 0 et 1)
 * \return Dérivée seconde en \a t
 */
cv::Point2d Courbe::computePtPrimePrime(double t)
{
    cv::Point2d res;
    res.x=_ddpolyX.compute(t);
    res.y=_ddpolyY.compute(t);
    return res;
}

/*!
 * \brief Retourne les points de controle de la courbe
 * \return Vecteur contenant les points de controle de la courbe
 */
std::vector<cv::Point2d> Courbe::pc() const
{
    return _pc;
}

/*!
 * \brief Fonction de debuggage qui affiche la courbe de Bézier
 * \param img Image sur laquelle écrire
 * \param color (optionnel) Couleur du dessin
 * \param withPC (optionnel) Indique s'il faut afficher les points de controle (oui par défaut)
 */
void Courbe::draw(cv::Mat &img, cv::Scalar color, bool withPC)
{
    // On commence par vérifier que l'image peut afficher le patch
    std::vector<cv::Point> pts; // La fonction boundingRect de OpenCV n'accepte que des cv::Point
    for(unsigned int i=0;i<_pc.size();i++)
        pts.push_back(_pc[i]);
    cv::Rect bb=cv::boundingRect(pts);
    if(bb.x<0||bb.y<0||bb.height+bb.y>=img.rows||bb.width+bb.x>=img.cols)
        return;

    // Si les points de controle doivent etre affichés
    if(withPC){
        for(unsigned int i=0;i<_pc.size()-1;i++){
            cv::circle(img,_pc[i],3,color,CV_FILLED);
            cv::line(img,_pc[i],_pc[i+1],color);
        }
        cv::circle(img,_pc[_pc.size()-1],3,color,CV_FILLED);
    }

    // On échantillonne la courbe en 1000 points (c'est une fonction de deboggage, ca devrait suffire
    for(double t=0;t<=1;t+=0.001)
        img.at<cv::Vec3b>(computePt(t))=cv::Vec3b(color[0],color[1],color[2]);
}

/*!
 * \brief Déplace le point de contrôle \æ ind dans la direction \a dir avec une amplitude \a scale
 * \param ind Indice du point de contrôle à déplacer
 * \param dir Vecteur directeur du mouvement
 * \param scale Amplitude du mouvement
 */
void Courbe::movePtCtrl(int ind, cv::Point2d dir, double scale)
{
    if(_pc.size()>ind){ // Si le point de controle existe
        _pc[ind]+=dir*scale; // On le déplace
        if(_withPoly)
            buildPoly();
    }
}

/*!
 * \brief Mesure la distance entre un point \a pt donné et la courbe de bézier.
 *
 * \note Cette fonction est basée sur https://hal.archives-ouvertes.fr/file/index/docid/518379/filename/Xiao-DiaoChen2007c.pdf
 * \param pt Point dont on cherche la distance à la courbe
 * \param t Indice du point de la courbe qui minimise cette distance
 * \return distance euclidienne entre le point et la courbe
 */
double Courbe::distToCurve(cv::Point2d pt,long double& t)
{
    if(t==0||t==1)return 0;

    // Calcul des polynomes définissant la courbe si pas encore fait
    if(!_polyOK)
        buildPoly();

    // Calcul du polynome g(t)=Poly'(t).(pt-poly(t))
    rm::Algebre::Polynome g=_dpolyX*(rm::Algebre::Polynome(&(pt.x),0)-_polyX)+_dpolyY*(rm::Algebre::Polynome(&(pt.y),0)-_polyY);
    // Sequence de Sturm pour identifier la position des racines


    std::vector<rm::Intervalles::Intervalle<int> > intervalles=g.sturmSequence(0,1,1);

    // On vérifie que chaque intervalle contient au plus une racine
    // Sinon, on applique de nouveau la séquence de Sturm pour subdiviser l'intervalle en question
    // jusqu'à obtenir une racine par intervalle
    bool stop;
    do{
        stop=true;
        for(unsigned int i=0;i<intervalles.size();i++){
            if(intervalles[i].data()>1){
                stop=false;
                std::vector<rm::Intervalles::Intervalle<int> > newIntervalles;
                newIntervalles=g.sturmSequence(intervalles[i].borneInf(),
                                               intervalles[i].borneSup(),
                                               0.5*(intervalles[i].borneSup()-intervalles[i].borneInf()));
                for(unsigned int j=0;j<intervalles.size();j++){
                    if(i!=j)
                        newIntervalles.push_back(intervalles[j]);
                }
                intervalles=newIntervalles;
                i=-1;
            }
        }
    }while(!stop);

    // identification des intervalles intéressants (où le zéro de la fonction g correspond effectivement
    // à un minimum local de la distance au point
    // On s'appuie sur le fait que g(u)=alpha*f'(u), où f(u) est la distance entre le point pt et le point
    // de la courbe de coordonnées u, avec alpha<0. Donc quand g>0, f'<0 et inversement
    std::vector<rm::Intervalles::Intervalle<int> > intervallesTmp;
    for(unsigned int i=0;i<intervalles.size();i++){
        if(g.compute(intervalles[i].borneInf())>0&&g.compute(intervalles[i].borneSup())<0)
            intervallesTmp.push_back(intervalles[i]);
    }
    intervalles=intervallesTmp;

    // On sait maintenant que, sur les intervalles qui restent, la fonction g est monotone, et passe par 0
    // On converge donc vers une bonne approximation de la racine en divisant à chaque itération par 2 l'intervalle
    // suivant le signe de la fonction g en son milieu.
    for(unsigned int i=0;i<intervalles.size();i++){
        while(intervalles[i].largeur()>0.00000001){



            long double gMilieu=g.compute(intervalles[i].milieu());
            if(gMilieu==0){
                intervalles[i].set(intervalles[i].milieu(),intervalles[i].milieu(),1);
            }
            else if(gMilieu>0){
                intervalles[i].setBorneInf(intervalles[i].milieu());
            }
            else{
                intervalles[i].setBorneSup(intervalles[i].milieu());
            }
        }
    }

    double bestDist=10000000;
    int best;
    for(unsigned int i=0;i<intervalles.size();i++){
        double dist=cv::norm(pt-computePtPoly(intervalles[i].milieu()));
        if(dist<bestDist){
            bestDist=dist;
            best=i;
        }
    }


    t=intervalles[best].milieu();
    return bestDist;

}

/*!
 * \brief Met à disposition le polynome suivant X définissant la ocurbe
 * \return Polynome suivant x
 */
Algebre::Polynome Courbe::polyX(){
    return _polyX;
}

/*!
 * \brief Met à disposition le polynome suivant Y définissant la ocurbe
 * \return Polynome suivant y
 */
Algebre::Polynome Courbe::polyY(){
    return _polyY;
}

/*!
 * \brief Exprime la courbe de Bézier sous forme d'un polynome
 */
void Courbe::buildPoly()
{
    double pcX[_pc.size()],pcY[_pc.size()];
    for(unsigned int i=0;i<_pc.size();i++){
        pcX[i]=_pc[i].x;
        pcY[i]=_pc[i].y;
    }

    if(_pc.size()==4){
        double coefsX[4];
        coefsX[0]=pcX[0];
        coefsX[1]=-3*pcX[0]+3*pcX[1];
        coefsX[2]=3*pcX[0]-6*pcX[1]+3*pcX[2];
        coefsX[3]=3*pcX[1]+pcX[3]-pcX[0]-3*pcX[2];
        _polyX.set(coefsX,3);
        double coefsY[4];
        coefsY[0]=pcY[0];
        coefsY[1]=-3*pcY[0]+3*pcY[1];
        coefsY[2]=3*pcY[0]-6*pcY[1]+3*pcY[2];
        coefsY[3]=3*pcY[1]+pcY[3]-pcY[0]-3*pcY[2];
        _polyY.set(coefsY,3);
    }
    else{
        _polyX=deCasteljauPoly(pcX,_pc.size());
        _polyY=deCasteljauPoly(pcY,_pc.size());
    }


    _dpolyX=_polyX.derivate();
    _dpolyY=_polyY.derivate();
    _ddpolyX=_dpolyX.derivate();
    _ddpolyY=_dpolyY.derivate();

    _withPoly=true;
    _polyOK=true;
}

/*!
 * \brief Algorithme récursif de DeCasteljau pour l'expression de la courbe sous forme de polygone
 *
 * Il s'agit d'une méthode privée de la structure, appelée par la méthode privée buildPoly
 *
 * \param pc points de controle considérés à une itération donnée
 * \return Polynome à une itération donnée
 */
Algebre::Polynome Courbe::deCasteljauPoly(double * Pc,int n)
{

    std::vector<double> res;
    if(n==1){
        return(rm::Algebre::Polynome(&(Pc[0]),0));
    }

    double poly1[2],poly2[2];
    poly1[0]=1;
    poly1[1]=-1;
    poly2[0]=0;
    poly2[1]=1;

    return rm::Algebre::Polynome(poly1,1)*deCasteljauPoly(Pc,n-1)+rm::Algebre::Polynome(poly2,1)*deCasteljauPoly(&(Pc[1]),n-1);
}

/*!
 * \brief Algorithme récursif de DeCasteljau pour le calcul des coordonnées d'un point de la courbe
 *
 * Il s'agit d'une méthode privée de la structure, appelée par la méthode publique computePt
 *
 * \param pc points de controle considérés, à une itération donnée
 * \param t position du point recherché sur la courbe (entre 0 et 1)
 * \return coordonnées du point recherché
 */
cv::Point2d Courbe::deCasteljau(std::vector<cv::Point2d> Pc, double t)
{
    // Si un seul point de controle, la récursivité s'achève
    if(Pc.size()==1)
        return Pc[0];

    // v1 = pc - son dernier élément
    // v2 = pc - son premier élément
    std::vector<cv::Point2d> v1,v2;
    v1.insert(v1.begin(),Pc.begin(),Pc.begin()+Pc.size()-1);
    v2.insert(v2.begin(),Pc.begin()+1,Pc.begin()+Pc.size());
    return (1-t)*deCasteljau(v1,t)+t*deCasteljau(v2,t);
}

/*!
 * \brief Approxime un nuage de points par un ensemble de courbes de Bézier cubique
 *
 * Les courbes générées vérifient une contrainte de connectivité G1
 * \note fortement inspiré de Philip J. Schneider's "Algorithm for Automatically Fitting Digitized Curves" from the book "Graphics Gems"
 * \param pts Nuage de points à approximer (le premier et le dernier sont les extrémités de la courbe)
 * \param thres Distance maximale entre un point et la courbe
 * \return Vecteur de courbes de bézier définissant la courbe totale
 */
std::vector<Courbe> fitCubicCurves(std::vector<cv::Point2d> pts, double thres)
{

    std::vector<Courbe> res;
    Courbe curve;
    // Chord-length method pour la première estimation de la courbe
    double lCordes[pts.size()-1];
    double som=0;
    for(unsigned int i=0;i<pts.size()-1;i++){
        lCordes[i]=cv::norm(pts[i]-pts[i+1]);
        som+=lCordes[i];
    }
    long double t[pts.size()];
    t[0]=0;
    for(unsigned int i=1;i<pts.size()-1;i++){
        t[i]=t[i-1]+lCordes[i]/som;
    }
    t[pts.size()-1]=1;

    struct Bernstein{
        long double compute(int i,long double t){
            long double res;
            switch(i){
            case 0:res= (1-t)*(1-t)*(1-t);break;
            case 1:res= 3*t*(1-t)*(1-t);break;
            case 2:res= 3*t*t*(1-t);break;
            case 3:res= t*t*t;break;
            default:return 0;
            }
            return res;
        }
    };
    Bernstein bernstein;

    long double theta1,theta2;

    // Calcul des tangentes au premier et dernier points
    cv::Point2d proj;
    rm::Geometrie::Distances::pointToSegment2D(pts[1],pts[0],pts[2],proj);
    cv::Point2d dir=2*pts[1]-proj;
    cv::Point2d tanDeb;
    tanDeb.x=(dir.x-pts[0].x)/cv::norm(dir-pts[0]);
    tanDeb.y=(dir.y-pts[0].y)/cv::norm(dir-pts[0]);
    theta1=atan2((long double)tanDeb.y,(long double)tanDeb.x);
    cv::Point2d tanFin;
    rm::Geometrie::Distances::pointToSegment2D(pts[pts.size()-2],pts[pts.size()-1],pts[pts.size()-3],proj);
    dir=2*pts[pts.size()-2]-proj;
    tanFin.x=(-pts[pts.size()-1].x+dir.x)/cv::norm(pts[pts.size()-1]-dir);
    tanFin.y=(-pts[pts.size()-1].y+dir.y)/cv::norm(pts[pts.size()-1]-dir);
    theta2=atan2((long double)tanFin.y,(long double)tanFin.x);

    long double B0,B1,B2,B3;

    cv::Point2d D1,D2;

    std::vector<cv::Point2d> pc(4);
    pc[0]=pts[0];
    pc[3]=pts[pts.size()-1];


    for(int iter=0;iter<20000;iter++){

        // A11
        long double A11=0;
        for(int i=0;i<pts.size();i++){
            B1=bernstein.compute(1,t[i]);


            A11+=B1*B1;

            std::cout<<A11<<" ";
        }

        // A22
        long double A22=0;
        for(int i=0;i<pts.size();i++){
            B2=bernstein.compute(2,t[i]);

            A22+=B2*B2;
        }

        // A12 A21
        long double A12=0,A21=0;
        for(int i=0;i<pts.size();i++){
            B1=bernstein.compute(1,t[i]);
            B2=bernstein.compute(2,t[i]);

            A21+=cos(theta1-theta2)*B1*B2;
            A12+=cos(theta1-theta2)*B1*B2;
        }

        // X1 / X2
        long double X1=0;
        long double X2=0;
        for(int i=0;i<pts.size();i++){
            B0=bernstein.compute(0,t[i]);
            B1=bernstein.compute(1,t[i]);
            B2=bernstein.compute(2,t[i]);
            B3=bernstein.compute(3,t[i]);

            long double tmpx=pts[i].x-pc[0].x*(B0+B1)-pc[3].x*(B2+B3);
            long double tmpy=pts[i].y-pc[0].y*(B0+B1)-pc[3].y*(B2+B3);

            X1+=tmpx*cos(theta1)*B1+tmpy*sin(theta1)*B1;
            X2+=tmpx*cos(theta2)*B2+tmpy*sin(theta2)*B2;
        }
        std::cout<<std::endl;

        long double den=A11*A22-A12*A21;
        long double num1=X1*A22-X2*A12;
        long double num2=A11*X2-A21*X1;

        long double alpha1=num1/den;
        long double alpha2=num2/den;

        pc[1].x=pc[0].x+cos(theta1)*alpha1;
        pc[1].y=pc[0].y+sin(theta1)*alpha1;

        pc[2].x=pc[3].x+cos(theta2)*alpha2;
        pc[2].y=pc[3].y+sin(theta2)*alpha2;

        curve.set(pc);


        // Convergence
        double som=0,somPrev=0,som2=0;
        for(int i=0;i<pts.size();i++){
            somPrev+=cv::norm(pts[i]-curve.computePt(t[i]));
            som+=curve.distToCurve(pts[i],t[i]);
        }

        std::cout<<somPrev<<" "<<som<<" "<<alpha1<<" "<<alpha2<<std::endl;
        cv::Mat img(500,500,CV_8UC3);
        for(int i=0;i<pts.size();i++){
            cv::line(img,pts[i],curve.computePtPoly(t[i]),cv::Scalar(0,255,0));
            cv::circle(img,pts[i],3,cv::Scalar(255,0,0));
        }
        //            cv::circle(img,pts[furtherPt],3,cv::Scalar(0,0,255));
        curve.draw(img,cv::Scalar(255,255,255));
        cv::imshow("img",img);cv::waitKey();

    }
    res.push_back(curve);

    return res;


}

}
}
}
