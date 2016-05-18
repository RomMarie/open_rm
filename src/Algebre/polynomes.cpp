#include <open_rm/Algebre/polynomes.h>

namespace rm{
namespace Algebre{

using namespace std;

/*!
 * \brief Effectue la division euclidienne d'un polynome \a N par un autre \a D
 *
 * La fonction retourne \a q et \a r tels que N=qD+r
 * \note basé sur https://fr.wikipedia.org/wiki/Division_d'un_polyn%C3%B4me et https://rosettacode.org/wiki/Polynomial_long_division#C.2B.2B
 *
 * \param den Polynome servant de dénominateur
 * \param Q Polynome résultat de la division euclidienne de l'instance par D
 * \param R Polynome correspondant au reste de la division euclidienne de l'instance par D
 */
void Polynome::divisionPolynomiale(const Polynome& den, Polynome& Q, Polynome& R)
{
    // degrés des différents polynomes utilisés
    int dN=degre();
    int dD=den.degre();
    int dq=dN-dD;
    int dr=dD-1;
    int dd=dN;
    // variable de boucle
    int i;
    // Déclaration des structures utilisées
    double N[dN+1];
    double D[dD+1];
    double q[dq+1];
    double r[dr+1];
    double d[dd+1];
    // Remplissage des tableaux utilisés
    coefs(N,dN);
    den.coefs(D,dD);

    // Déroulement de l'algorihme
    if(dN>=dD){
        while(dN>=dD){
            for(i=0;i<=dN;i++){
                d[i]=0;
            }

            for(i=0;i<=dD;i++){
                d[i+dN-dD]=D[i];
            }
            dd = dN;
            q[dN-dD] = N[dN]/d[dd];
            for(i=0;i<=dd;i++){
                d[i]=d[i]*q[dN-dD];
            }
            for(i=0;i<=dN;i++){
                N[i]=N[i]-d[i];
            }
            dN--;
        }
    }

    for(i=0;i<=dN;i++){
        r[i]=N[i];
    }

    while(q[dq]==0&&dq>0)
        dq--;

    while(r[dr]==0&&dr>0)
        dr--;

    Q.set(q,dq);
    R.set(r,dr);

}

/*!
 * \brief Identifie le nombre de racines réelles du polynome entre chaque intervalle de dimension \a step entre \a start et \a end
 * \note voir https://en.wikipedia.org/wiki/Sturm's_theorem pour plus les détails de la méthode
 * \param start Valeur de début de l'analyse du polynome
 * \param end Valeur de fin de l'analyse du polynome
 * \param step Pas des intervalles considérés
 * \return Vecteur retournant pour chaque intervalle le nombre de racines réelles du polynome s'y trouvant
 */
std::vector<rm::Intervalles::Intervalle<int> > Polynome::sturmSequence(double start, double end, double step)
{
    // Construction de la séquence de Sturm
    if(_sturmSeq.size()==0){
        _sturmSeq.push_back(*this);
        _sturmSeq.push_back(derivate());
        Polynome s;
        do{
            s=_sturmSeq[_sturmSeq.size()-2]%_sturmSeq[_sturmSeq.size()-1];
            if(s.degre()>=0){
                s.invert();
                _sturmSeq.push_back(s);
            }
        }while(s.coefs().size()>1);
    }


    // Evaluation de S pour chaque borne de chaque intervalle (start+n*step)

    int tailleSeq=_sturmSeq.size();
    int nBornes=floor((end-start)/step+1);

    double Ss[nBornes][tailleSeq];

    int n=0;
    for(double i=start;i<=end;i+=step){
        for(unsigned int j=0;j<tailleSeq;j++){
            Ss[n][j]=_sturmSeq[j].compute(i);
        }
        n++;
    }

    // Calcul du nombre de changement de signe pour chaque borne
    int nSwap[nBornes];
    for(unsigned int i=0;i<nBornes;i++){
        nSwap[i]=0;
        for(unsigned int j=1;j<tailleSeq;j++){
            if(Ss[i][j-1]/Ss[i][j]<0)
                nSwap[i]++;
        }
    }

    // Calcul du nombre de racines dans chaque intervalle
    int nRacines[nBornes-1];
    for(unsigned int i=1;i<nBornes;i++){
        if(nSwap[i-1]-nSwap[i]>0)
            nRacines[i-1]=nSwap[i-1]-nSwap[i];
        else
            nRacines[i-1]=0;
    }

    // Mise en forme et envoi du résultat
    std::vector<rm::Intervalles::Intervalle<int> > res;
    for(unsigned int i=0;i<nBornes-1;i++){
        if(nRacines[i]>0)
            res.push_back(rm::Intervalles::Intervalle<int>(start+i*step,start+(i+1)*step,nRacines[i]));
    }
    return res;
}

/*!
 * \brief Constructeur par défaut
 */
Polynome::Polynome()
{

}

/*!
 * \brief Constructeur principal
 * \param coefs Coefficients du polynome
 */
Polynome::Polynome(std::vector<double> coefs)
{
    set(coefs);
}

/*!
 * \brief Second contructeur principal (autre format d'entrée)
 * \param coefs Coefficients du polynome
 * \param degre Degré du polynome
 */
Polynome::Polynome(double *coefs, int degre)
{

    if(_coefs.size()!=degre+1){
        _coefs.resize(degre+1);
    }
    for(unsigned int i=0;i<=degre;i++){
        _coefs[i]=coefs[i];
    }
}

/*!
 * \brief Constructeur de recopie
 * \param poly Polynome à copier
 */
Polynome::Polynome(const Polynome &poly)
{
    _coefs=poly.coefs();
}

/*!
 * \brief Applique un facteur -1 à chaque coefficient du polynome
 */
void Polynome::invert()
{
    int deg=degre();
    for(int i=0;i<=deg;i++){
        _coefs[i]*=-1;
    }
}

/*!
 * \brief Calcule la fonction f(x)=Polynome
 * \param x Valeur désirée de l'inconnue
 * \return Valeur du polynome en x
 */
double Polynome::compute(double x)
{
    double res=0;
    int deg=degre();
    for(unsigned int i=0;i<=deg;i++){
        double power=1;
        for(int j=0;j<i;j++)
            power*=x;
        res+=_coefs[i]*power;
    }
    return res;
}

/*!
 * \brief Calcule l'expression de la dérivée du polynome
 * \return Polynome correspondant à la dérivée du polynome
 */
Polynome Polynome::derivate()
{
    if(degre()<1){
        return Polynome();
    }

    int degRes=degre()-1;
    double res[degRes+1];

    for(int i=0;i<=degRes;i++){
        res[i]=(i+1)*_coefs[i+1];
    }

    return(Polynome(res,degRes));
}

/*!
 * \brief Surcharge de l'opérateur + (effectue l'addition de deux polynomes)
 * \param poly Polynome à additionner
 * \return Polynome résultat de l'addition
 */
Polynome Polynome::operator+(const Polynome &poly)
{

    int degPoly2=poly.degre();
    double coefsPoly2[degPoly2+1];
    poly.coefs(coefsPoly2,degPoly2);

    int degRes;
    if(degPoly2+1>_coefs.size()){
        degRes=degPoly2;
    }
    else{
        degRes=degre();
    }
    double coefsRes[degRes+1];
    for(int i=0;i<=degRes;i++){
        coefsRes[i]=0;
        if(i<=degPoly2)
            coefsRes[i]+=coefsPoly2[i];
        if(i<=degre())
            coefsRes[i]+=_coefs[i];
    }

    for(int i=degRes;i>=0;i--){
        if(coefsRes[i]==0)
            degRes--;
        else
            break;
    }

    if(degRes==-1)return Polynome();
    return Polynome(coefsRes,degRes);
}

/*!
 * \brief Surcharge de l'opérateur - (effectue la soustraction de deux polynomes)
 * \param poly Polynome à additionner
 * \return Polynome résultat de l'addition
 */
Polynome Polynome::operator-(const Polynome &poly)
{
    int degPoly2=poly.degre();
    double coefsPoly2[degPoly2+1];
    poly.coefs(coefsPoly2,degPoly2);

    int degRes;
    if(degPoly2>degre()){
        degRes=degPoly2;
    }
    else{
        degRes=degre();
    }
    double coefsRes[degRes+1];
    for(int i=0;i<=degRes;i++){
        coefsRes[i]=0;
        if(i<=degPoly2)
            coefsRes[i]-=coefsPoly2[i];
        if(i<=degre())
            coefsRes[i]+=_coefs[i];
    }

    for(int i=degRes;i>=0;i--){
        if(coefsRes[i]==0){
            degRes--;
        }
        else
            break; // On retire des éléments jusqu'à avoir un coefficient non nul
    }

    if(degRes==-1)return Polynome();
    return Polynome(coefsRes,degRes);
}

/*!
 * \brief Surcharge de l'opérateur * (multiplie deux polynomes entre eux)
 * \param poly
 * \return
 */
Polynome Polynome::operator*(const Polynome &poly)
{
    int degrePoly2=poly.degre();
    int dim=degrePoly2+_coefs.size()-1; // degré1 + degré2
    double res[dim];
    for(int i=0;i<dim;i++)
        res[i]=0;

    double coefs[degrePoly2+1];
    poly.coefs(coefs,degrePoly2);
    for(unsigned int i=0;i<=degrePoly2;i++){
        for(unsigned int j=0;j<_coefs.size();j++){
            res[i+j]+=(long double)coefs[i]*(long double)_coefs[j];
        }
    }
    return Polynome(res,dim);
}

/*!
 * \brief Surcharge de l'opérateur / (quotient de la division euclidienne entre deux polynomes)
 * \param poly Polynome correspondant au dénominateur
 * \return Polynome correspondant à la division entière par \a poly
 */
Polynome Polynome::operator/(const Polynome &poly)
{
    Polynome R;
    Polynome Q;
    divisionPolynomiale(poly,Q,R);
    return Q;
}

/*!
 * \brief Surcharge de l'opérateur % (reste de la division polynomiale)
 * \param poly Dénominateur de la division eutre deux polynomes
 * \return reste de la division entre deux polynomes
 */
Polynome Polynome::operator%(const Polynome &poly)
{
    Polynome R;
    Polynome Q;
    divisionPolynomiale(poly,Q,R);
    return R;
}

/*!
 * \brief Surcharge de l'opérateur << (pour affichage via cout)
 * \param os Stream d'entrée
 * \param poly Polynome à afficher
 * \return Stream de sortie
 */
std::ostream & operator<<(std::ostream &os, const Polynome &poly)
{
    os<<"[";
    for(int i=poly.coefs().size()-1;i>=0;i--){
        if(poly.coefs()[i]!=0){
            if(i!=poly.coefs().size()-1){
                if(poly.coefs()[i]<0)
                    os<<" - ";
                else
                    os<<" + ";
            }
            else if(poly.coefs()[i]<0)
                os<<"-";

            if(abs(poly.coefs()[i])==1){
                if(i>1)
                    os<<"x^"<<i;
                else if(i==1)
                    os<<"x";
                else
                    os<<"1";
            }
            else{
                if(i>1)
                    os<<abs(poly.coefs()[i])<<"x^"<<i;
                else if(i==1)
                    os<<abs(poly.coefs()[i])<<"x";
                else
                    os<<abs(poly.coefs()[i]);
            }
        }
    }
    os<<"]";
    return os;
}

/*!
 * \brief Attribue de nouveaux coefficients au polynome
 * \param coefs Coefficients du polynome
 */
void Polynome::set(std::vector<double> coefs)
{
    _coefs=coefs;
}

/*!
 * \brief Attribue de nouveaux coefficients au polynome
 * \param coefs Coefficients du polynome
 * \param degre Degré du polynome
 */
void Polynome::set(double *coefs, int degre)
{
    if(_coefs.size()!=degre+1){
        _coefs.resize(degre+1);
    }
    for(unsigned int i=0;i<_coefs.size();i++){
        _coefs[i]=coefs[i];
    }
}

/*!
 * \brief Accesseur aux coefficients du polynome porté par l'instance
 * \return vecteur de double contenant les coefficients par ordre croissant
 */
std::vector<double> Polynome::coefs() const
{
    return _coefs;
}

/*!
 * \brief Récupère les coefficients du polynome sous la forme d'un tableau
 * \param coef Pointeur vers le tableau chargé de récupérer les coefficients
 * \param degre Degré du polynome (indispensable pour éviter les fuites mémoires)
 */
void Polynome::coefs(double *coef, int degre) const
{
    if(degre!=_coefs.size()-1)return;

    for(unsigned int i=0;i<_coefs.size();i++){
        coef[i]=_coefs[i];
    }
}

/*!
 * \brief Indique le degré du polynome
 * \return Degré du polynome
 */
int Polynome::degre() const
{
    return _coefs.size()-1;
}

}
}
