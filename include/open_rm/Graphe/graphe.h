#ifndef GRAPHE_H
#define GRAPHE_H

#include <boost/graph/adjacency_list.hpp>

// La gestion des graphes est basée sur Boost Graph Library (BGL)
using namespace boost;

enum vertex_properties_t { vertex_properties };
enum edge_properties_t { edge_properties };
namespace boost {
BOOST_INSTALL_PROPERTY(vertex, properties);
BOOST_INSTALL_PROPERTY(edge, properties);
}

namespace rm{
namespace Graphe{

/*! \brief Wrapper générique de graphe non orienté.
 *
 * VERTEXPROPERTIES et EDGEPROPERTIES doivent être des structures
 * qui contiennent respectivement les informations portées par
 * les noeuds et les aretes
 */
template < typename VERTEXPROPERTIES, typename EDGEPROPERTIES >
class GrapheNO
{
public:

    typedef adjacency_list<
    vecS,
    vecS,
    undirectedS,
    property<vertex_properties_t, VERTEXPROPERTIES>,
    property<edge_properties_t, EDGEPROPERTIES>
    > GraphContainer; ///< Conteneur de graphe

    // Ensemble de typedef pour simplifier les notations
    typedef typename graph_traits<GraphContainer>::vertex_descriptor Vertex; ///< Accesseur vertex
    typedef typename graph_traits<GraphContainer>::edge_descriptor Edge; ///< Accesseur arete
    typedef std::pair<Edge, Edge> EdgePair; ///< Paire d'arete
    typedef typename graph_traits<GraphContainer>::vertex_iterator vertex_iter; ///< itérateur de vertex
    typedef typename graph_traits<GraphContainer>::edge_iterator edge_iter; ///< itérateur d'arete
    typedef typename graph_traits<GraphContainer>::adjacency_iterator adjacency_iter; ///< itérateur d'adjacence
    typedef typename graph_traits<GraphContainer>::out_edge_iterator out_edge_iter; ///< itérateur d'arete sortante
    typedef typename graph_traits<GraphContainer>::degree_size_type degree_t; ///< degré topologique
    typedef std::pair<adjacency_iter, adjacency_iter> adjacency_vertex_range_t; ///< range d'adjacence
    typedef std::pair<out_edge_iter, out_edge_iter> out_edge_range_t; ///< range d'arete sortante
    typedef std::pair<vertex_iter, vertex_iter> vertex_range_t; ///< range de vertex
    typedef std::pair<edge_iter, edge_iter> edge_range_t; ///< range d'arete

    // -------------------------------------- Constructeurs / dstructeurs -------------------------------------------- //

    /*! \brief Constructeur par défaut de la classe
     * Le graphe généré est vide
     */
    GrapheNO()
    {}

    /*! \brief Constructeur de recopie
     */
    GrapheNO(const GrapheNO& g) :  graph(g.graph)
    {}


    /*! \brief Operateur de recopie
     */
    GrapheNO& operator=(const GrapheNO &rhs)
    {
        graph = rhs.graph;
        return *this;
    }

    /*! \brief Destructeur par défaut
     */
    virtual ~GrapheNO()
    {}

    // -------------------------------------- Accès aux propriétés des éléments -------------------------------------------- //
    // properties(v) -> structure vers les informations portées par le vertex v
    // properties(e) -> structure vers les informations portées par l'edge e
    /*!
     * \brief Propriétés d'un vertex
     * \param v Vertex considéré
     * \return  Propriétés du vertex
     */
    VERTEXPROPERTIES& properties(const Vertex& v)
    {
        typename property_map<GraphContainer, vertex_properties_t>::type param = get(vertex_properties, graph);
        return param[v];
    }

    /*!
     * \brief Propriétés d'une arete
     * \param e Arete considérée
     * \return Propriétés de l'arete considérée
     */
    EDGEPROPERTIES& properties(const Edge& e)
    {
        typename property_map<GraphContainer, edge_properties_t>::type param = get(edge_properties, graph);
        return param[e];
    }

    // --------------------------------------- Remplissage ----------------------------------------------------------- //



    /*! \brief Fonction qui ajoute un noeud au graphe en lui attribuant
     * les propriétés passées en argument.
     *
     * \param[in] prop attributs du noeud créé
     * \return iterator vers le noeud ainsi créé
     */
    Vertex addVertex(const VERTEXPROPERTIES& prop)
    {
        // On ajoute le noeud au graphe
        Vertex v = add_vertex(graph);

        // On lui attribue les propriétés passées en argument
        properties(v) = prop;

        // Et on renvoit l'itérator généré à la création du noeud
        return v;
    }
    /*! \brief Fonction qui ajoute une arête à la représentation
     * et lui attribue les propriétés désirées.
     * \param[in] ind1,ind2 indice des noeuds connectés par l'arête (iterators)
     * \param[in] prop propriétés de l'arête ajoutée
     * \return iterator vers l'arête créée
     */
    Edge addEdge(int ind1, int ind2, const EDGEPROPERTIES& prop)
    {
        // On recherche les deux noeuds dans le graphe
        Vertex v1=boost::vertex(ind1,graph);
        Vertex v2=boost::vertex(ind2,graph);

        // On ajoute l'arete entre les noeuds v1 et v2
        Edge addedEdge = add_edge(v1, v2, graph).first;

        // On lui attribue les propriétés passées en argument
        properties(addedEdge) = prop;

        // et on retoune l'iterator récupéré lors de la création
        return addedEdge;
    }

    // ------------------------------------------------ Getters ------------------------------------------------------ //

    /*! \brief Accès aux informations d'un noeud du graphe
     * \param[in] ind indice du noeud à lire
     * \return propriétés portées par le noeud ind
     */
    VERTEXPROPERTIES getVertex(int ind)
    {
        vertex_range_t noeudsIter=boost::vertices(graph);
        typename property_map<GraphContainer, vertex_properties_t>::type param = get(vertex_properties, graph);
        return param[*(noeudsIter.first+ind)];
    }

    /*! \brief Récupère les propriétés de l'ensemble des noeuds du graphe
     * \return vecteurs contenant les propriétés de chaque noeud
     */
    std::vector<VERTEXPROPERTIES> getAllVertex()
    {
        std::vector<VERTEXPROPERTIES> res;
        vertex_range_t noeudsIter=boost::vertices(graph);
        typename property_map<GraphContainer, vertex_properties_t>::type param = get(vertex_properties, graph);

        for(;noeudsIter.first!=noeudsIter.second;++noeudsIter.first)
            res.push_back(param[*(noeudsIter.first)]);
        return res;
    }

    /*! \brief Accès aux informations d'une arête du graphe
     * \param[in] ind indice de l'arête à lire
     * \return propriétés portées par l'arête ind
     */
    EDGEPROPERTIES& getEdge(int ind)
    {
        typename property_map<GraphContainer, edge_properties_t>::type param = get(edge_properties, graph);

        std::pair<edge_iter,edge_iter> ep = edges( graph );
        for(int i=0;i<ind;i++)
            ep.first++;
        return param[*(ep.first)];
    }

    /*! \brief Fonction qui renvoit l'adjacency list porteuse du graph
     */
    const GraphContainer& getGraph() const
    {
        return graph;
    }

    /*! \brief Fonction qui renvoit le nombre de noeuds du graphe
     */
    int getVertexCount() const
    {
        return num_vertices(graph);
    }

    /*! \brief Fonction qui renvoit le nombre d'arêtes du graphe
     */
    int getEdgeCount() const
    {
        int n=0;
        std::pair<edge_iter, edge_iter> ep;
        for (ep = edges(graph); ep.first != ep.second; ++ep.first)
            n++;
        return n;
    }

    // -------------------------------------------- Setters ------------------------------------------------------------ //
    /*! \brief modifie les attributs portés par un noeud du graphe
     * \param[in] prop nouvelles propriétés du noeud
     * \param[in] ind indice du noeud à modifier
     */
    void setVertex(const VERTEXPROPERTIES& prop,int ind)
    {
        // On récupère le pointeur vers le noeud
        Vertex v=boost::vertex(ind,graph);

        typename property_map<GraphContainer, vertex_properties_t>::type param = get(vertex_properties, graph);
        param[v]=prop;
        // On lui attribue les propriétés passées en argument
//        properties(v) = prop;
    }

    /*! \brief modifie les attributs portés par une arête du graphe
     * \param[in] prop nouvelles propriétés de l'arête
     * \param[in] ind indice de l'arête à modifier
     */
    void setEdge(const EDGEPROPERTIES& prop,int ind)
    {
        // On récupère le pointeur vers l'arête
        Edge e=boost::edge(ind,graph);

        // On lui attribue les propriétés passées en argument
        properties(e) = prop;
    }

    // -------------------------------------------- Manipulation du graphe --------------------------------------------- //

    /*!
     * Fonction qui supprime tous les noeuds et arêtes du graphe
     */
    void clearGraph()
    {
        graph.clear();
    }


protected:
    GraphContainer graph; ///< structure du graphe enregistré
};


}
}
#endif // GRAPHE_H
