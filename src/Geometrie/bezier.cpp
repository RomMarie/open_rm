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
cv::Point2d Courbe::computePt(double t)
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
double Courbe::distToCurve(cv::Point2d pt,double& t)
{
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
cv::Point2d Courbe::deCasteljau(std::vector<cv::Point2d> Pc, float t)
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
    Courbe curve,curve1m,curve1p,curve2m,curve2p;

    // Chord-length method pour la première estimation de la courbe
    double lCordes[pts.size()-1];
    double som=0;
    for(unsigned int i=0;i<pts.size()-1;i++){
        lCordes[i]=cv::norm(pts[i]-pts[i+1]);
        som+=lCordes[i];
    }
    double t[pts.size()];
    t[0]=0;
    for(unsigned int i=1;i<pts.size()-1;i++){
        t[i]=t[i-1]+lCordes[i]/som;
    }
    t[pts.size()-1]=1;

    struct Bernstein{
        double compute(int i,double t){
            switch(i){
            case 0:return (1-t)*(1-t)*(1-t);break;
            case 1:return 3*t*(1-t)*(1-t);break;
            case 2:return 3*t*t*(1-t);break;
            case 3:return t*t*t;break;
            default:return 0;
            }
        }
    };
    Bernstein bernstein;

    double theta1,theta2;

    // Calcul des tangentes au premier et dernier points
    cv::Point2d proj;
    rm::Geometrie::Distances::pointToSegment2D(pts[1],pts[0],pts[2],proj);
    cv::Point2d dir=2*pts[1]-proj;
    cv::Point2d tanDeb;
    tanDeb.x=(dir.x-pts[0].x)/cv::norm(dir-pts[0]);
    tanDeb.y=(dir.y-pts[0].y)/cv::norm(dir-pts[0]);
    theta1=atan2(tanDeb.y,tanDeb.x);
    cv::Point2d tanFin;
    rm::Geometrie::Distances::pointToSegment2D(pts[pts.size()-2],pts[pts.size()-1],pts[pts.size()-3],proj);
    dir=2*pts[pts.size()-2]-proj;
    tanFin.x=(pts[pts.size()-1].x-dir.x)/cv::norm(pts[pts.size()-1]-dir);
    tanFin.y=(pts[pts.size()-1].y-dir.y)/cv::norm(pts[pts.size()-1]-dir);
    theta2=atan2(tanFin.y,tanFin.x);

    double S=0,S1m=0,S1p=0,S2m=0,S2p=0;

    long double B0,B1,B2,B3;

    long double I1=0,I2=0,II1=0,II2=0,III1=0,III2=0;
    double I11m=0,I21m=0,II11m=0,II21m=0,III11m=0,III21m=0;
    double I11p=0,I21p=0,II11p=0,II21p=0,III11p=0,III21p=0;
    double I12m=0,I22m=0,II12m=0,II22m=0,III12m=0,III22m=0;
    double I12p=0,I22p=0,II12p=0,II22p=0,III12p=0,III22p=0;

    cv::Point2d D1,D2;

    std::vector<cv::Point2d> pc(4),pc1m(4),pc1p(4),pc2m(4),pc2p(4);
    pc[0]=pts[0];
    pc[3]=pts[pts.size()-1];

    pc1m[0]=pc[0];  pc1p[0]=pc[0];
    pc2m[0]=pc[0];  pc2p[0]=pc[0];
    pc1m[3]=pc[3];  pc1p[3]=pc[3];
    pc2m[3]=pc[3];  pc2p[3]=pc[3];

    for(int iter=0;iter<20000;iter++){
        //std::cout<<iter<<std::endl;

        tanDeb=cv::Point2d(cos(theta1),sin(theta1));
        tanFin=cv::Point2d(cos(theta2),sin(theta2));
        double theta1m=theta1-0.00001;
        double theta1p=theta1+0.00001;
        double theta2m=theta2-0.00001;
        double theta2p=theta2+0.00001;

        I1=0;I2=0;II1=0;II2=0;III1=0;III2=0;
        D1=cv::Point2d(0,0);
        D2=cv::Point2d(0,0);
        for(int i=0;i<pts.size();i++){
            B0=bernstein.compute(0,t[i]);
            B1=bernstein.compute(1,t[i]);
            B2=bernstein.compute(2,t[i]);
            B3=bernstein.compute(3,t[i]);
            I1+=(long double)B1*(long double)B1;
            II2+=(long double)B2*(long double)B2;
            II1+=(long double)B1*(long double)B2;

            D1.x+=B1*(pts[i].x-pc[0].x*(B0+B1)-pc[3].x*(B2+B3));
            D1.y+=B1*(pts[i].y-pc[0].y*(B0+B1)-pc[3].y*(B2+B3));
            D2.x+=B2*(pts[i].x-pc[0].x*(B0+B1)-pc[3].x*(B2+B3));
            D2.y+=B2*(pts[i].y-pc[0].y*(B0+B1)-pc[3].y*(B2+B3));

        }

        I11m=I1;        I11p=I1;
        I12m=I1;        I12p=I1;

        II11m=II1*cos(theta1m-theta2);        II11p=II1*cos(theta1p-theta2);
        II12m=II1*cos(theta1-theta2m);        II12p=II1*cos(theta1-theta2p);
        II1*=(long double)cos(theta1-theta2);

        I2=II1;
        I21m=I2;        I21p=I2;
        I22m=I2;        I22p=I2;

        II21m=II2;  II21p=II2;
        II22m=II2;  II22p=II2;

        III1=(long double)cos(theta1)*D1.x+(long double)sin(theta1)*D1.y;
        III11m=cos(theta1m)*D1.x+sin(theta1m)*D1.y;
        III11p=cos(theta1p)*D1.x+sin(theta1p)*D1.y;
        III12m=III1;
        III12p=III1;

        III2=(long double)cos(theta2)*D2.x+(long double)sin(theta2)*D2.y;
        III21m=III2;
        III21p=III2;
        III22m=cos(theta2m)*D2.x+sin(theta2m)*D2.y;
        III22p=cos(theta2p)*D2.x+sin(theta2p)*D2.y;

        long double alpha1=((long double)III1*(long double)II2-(long double)III2*(long double)II1)/
                (long double)((long double)I1*(long double)II2-(long double)I2*(long double)II1);
        long double alpha2=((long double)I1*(long double)III2-(long double)I2*(long double)III1)/
                ((long double)I1*(long double)II2-(long double)I2*(long double)II1);

            std::cout<<" Ligne 1 : "<<I1<<" "<<II1<<" "<<III1<<" "<<alpha1<<" "<<tanDeb<<std::endl;
          std::cout<<" Ligne 2 : "<<I2<<" "<<II2<<" "<<III2<<" "<<alpha2<<" "<<tanFin<<std::endl;
            std::cout<<" Numerateurs : "<<(III1*II2-III2*II1)<<" "<<(I1*III2-I2*III1)<<std::endl;
          std::cout<<" Denominateur : "<<(I1*II2-I2*II1)<<std::endl;

        double alpha11m=(long double)(III11m*II21m-III21m*II11m)/(long double)(I11m*II21m-I21m*II11m);
        double alpha21m=(long double)(I11m*III21m-I21m*III11m)/(long double)(I11m*II21m-I21m*II11m);

        double alpha11p=(long double)(III11p*II21p-III21p*II11p)/(long double)(I11p*II21p-I21p*II11p);
        double alpha21p=(long double)(I11p*III21p-I21p*III11p)/(long double)(I11p*II21p-I21p*II11p);
        double alpha12m=(long double)(III12m*II22m-III22m*II12m)/(long double)(I12m*II22m-I22m*II12m);
        double alpha22m=(long double)(I12m*III22m-I22m*III12m)/(long double)(I12m*II22m-I22m*II12m);
        double alpha12p=(long double)(III12p*II22p-III22p*II12p)/(long double)(I12p*II22p-I22p*II12p);
        double alpha22p=(long double)(I12p*III22p-I22p*III12p)/(long double)(I12p*II22p-I22p*II12p);

        pc[1]=pc[0]+tanDeb*(double)alpha1;
        pc[2]=pc[3]+tanFin*(double)alpha2;

        pc1m[1]=pc1m[0]+cv::Point2d(cos(theta1m),sin(theta1m))*alpha11m;
        pc1p[1]=pc1p[0]+cv::Point2d(cos(theta1p),sin(theta1p))*alpha11p;
        pc2m[1]=pc2m[0]+cv::Point2d(cos(theta1m),sin(theta1m))*alpha12m;
        pc2p[1]=pc2p[0]+cv::Point2d(cos(theta1p),sin(theta1p))*alpha12p;

        pc1m[2]=pc1m[3]+cv::Point2d(cos(theta2m),sin(theta2m))*alpha21m;
        pc1p[2]=pc1p[3]+cv::Point2d(cos(theta2p),sin(theta2p))*alpha21p;
        pc2m[2]=pc2m[3]+cv::Point2d(cos(theta2m),sin(theta2m))*alpha22m;
        pc2p[2]=pc2p[3]+cv::Point2d(cos(theta2p),sin(theta2p))*alpha22p;

        curve.set(pc);
        curve1m.set(pc1m,false);
        curve2m.set(pc2m,false);
        curve1p.set(pc1p,false);
        curve2p.set(pc2p,false);

        double somMain=0;
        double som1m=0;
        double som2m=0;
        double som1p=0;
        double som2p=0;
        for(int i=0;i<pts.size();i++){

            //            std::cout<<t[i]<<" ";
            somMain+=cv::norm(pts[i]-curve.computePt(t[i]));
            som1m+=cv::norm(pts[i]-curve1m.computePt(t[i]));
            som1p+=cv::norm(pts[i]-curve1p.computePt(t[i]));
            som2m+=cv::norm(pts[i]-curve2m.computePt(t[i]));
            som2p+=cv::norm(pts[i]-curve2p.computePt(t[i]));
        }
        //        std::cout<<std::endl;

        double delta1=0,delta2=0;
        double coef=0.0000001;
        //  if((som1p<somMain&&som1m>somMain)||
        //          (som1p>somMain&&som1m<somMain))
        delta1=coef*(som1p-som1m)/0.00002;
        //  if((som2p<somMain&&som2m>somMain)||
        //          (som2p>somMain&&som2m<somMain))
        delta2=coef*(som2p-som2m)/0.00002;

        /*std::cout<<somMain<<std::endl;
        std::cout<<"Thetas bornes : "<<theta1m<<" "<<theta1p<<" "<<theta2m<<" "<<theta2p<<std::endl;
        std::cout<<"Sommes : "<<som1m<<" "<<som1p<<" "<<som2m<<" "<<som2p<<std::endl;
        std::cout<<"Alphas : "<<alpha1<<" "<<alpha2<<std::endl;
        std::cout<<"Deltas : "<<delta1<<" "<<delta2<<std::endl;
        std::cout<<"Thetas : "<<theta1<<" "<<theta2<<std::endl<<std::endl;*/
        if(0){//iter%10!=9){
          //  if(iter%10!=9){
            //theta1-=delta1;
            //theta2-=delta2;
        }
        else{
            // Newton Rhapson

            double last_t[pts.size()];
            double somLast=0;
            double somNext=0;


            std::cout<<"Thetas : "<<theta1<<" "<<theta2<<std::endl;
            std::cout<<"SomMain : "<<somMain<<std::endl;

//            for(double i=0.8;i<0.9;i+=0.001)
  //              std::cout<<i<<" "<<cv::norm(pts[pts.size()-2]-curve.computePtPoly(i))<<std::endl;

            for(int i=1;i<pts.size()-1;i++){

                std::cout<<"Distances : "<<cv::norm(pts[i]-curve.computePtPoly(t[i]))<<" ";
                std::cout<<curve.distToCurve(pts[i],t[i])<<" ";
                std::cout<<cv::norm(pts[i]-curve.computePtPoly(t[i]))<<std::endl;




/*
                cv::Point2d f=curve.computePtPoly(t[i]);
                cv::Point2d fPrime=curve.computePtPrime(t[i]);
                cv::Point2d fPrimePrime=curve.computePtPrimePrime(t[i]);


                cv::Point2d d=f-pts[i];

                somLast+=cv::norm(d);
                long double numerator=d.x*fPrime.x+d.y*fPrime.y;
                long double denomi=fPrime.x*fPrime.x+fPrime.y*fPrime.y+d.x*fPrimePrime.x+d.y*fPrimePrime.y;

                std::cout<<numerator<<" "<<denomi<<std::abortendl;
                last_t[i]=t[i];
                if(denomi!=0)
                    t[i]-=numerator/denomi;
                //              std::cout<<t[i]<<" ";

                cv::Point2d fNext=curve.computePtPoly(t[i]);
                somNext+=cv::norm(fNext-pts[i]);*/
            }

            std::cout<<std::endl;
            //            std::cout<<std::endl;

            //            std::cout<<somLast<<" "<<somNext<<std::endl;

            if(somNext>somLast){
                for(int i=0;i<pts.size();i++){
                    t[i]=last_t[i];
                }
                break;
            }
            //else if(somLast/somNext<1.01)
            //    break;

        }
        cv::Mat img(500,500,CV_8UC3);
        for(int i=0;i<pts.size();i++){
            cv::line(img,pts[i],curve.computePtPoly(t[i]),cv::Scalar(0,255,0));
            cv::circle(img,pts[i],3,cv::Scalar(255,0,0));
        }
        //            cv::circle(img,pts[furtherPt],3,cv::Scalar(0,0,255));
        curve.draw(img,cv::Scalar(255,255,255));
        curve1m.draw(img,cv::Scalar(255,0,0));
        curve1p.draw(img,cv::Scalar(0,255,0));
        curve2m.draw(img,cv::Scalar(255,255,0));
        curve2p.draw(img,cv::Scalar(0,255,255));
        cv::imshow("img",img);cv::waitKey();

    }
    res.push_back(curve);

    return res;


}

}
}
}
