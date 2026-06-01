#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <readosm.h>

#define RAYON_TERRE 6371000.0
#define HASH_SIZE 200003  

//fait le lien entre l'id d'une node et un id interne, plus facile à manipuler
typedef struct HashNode {
    long long osm_id; //id du fichier osm
    int index;
    struct HashNode *suiv;
} HashNode;

HashNode *tableHash[HASH_SIZE]; //table de hachage pour trouver nos indices de noeud

//represente les aretes, ou ways, de notre graphe
typedef struct Way {
    int to; //sommet de destination
    double distance;
    int vitesse;
    double temps; //temps théorique pour parcourir l'arête
    struct Way *suiv;
} Way;

Way **graphe = NULL; //structure du réseau
int graphe_size = 0;


long long *node_id = NULL; //table inverse de notre table d'hachage
double *node_lat = NULL;
double *node_lon = NULL;
int node_count = 0;
int node_capacite = 0;

//permet d'assigner un id à notre grand index de noeux osm
unsigned long hashFunction(long long id) {
    id ^= (id >> 33);
    id *= 0xff51afd7ed558ccdULL;
    id ^= (id >> 33);
    id *= 0xc4ceb9fe1a85ec53ULL;
    id ^= (id >> 33);
    return (unsigned long)(id % HASH_SIZE);
}

//verifie si memoire libre pour assigner un nouveau noeud
//sinon on realloc pour créer de l'espace
void assure_node_capacite() {
    if (node_count < node_capacite) {
        return;
    }

    node_capacite = (node_capacite == 0) ? 100000 : node_capacite * 2;

    node_id = realloc(node_id, node_capacite * sizeof(long long));
    node_lat = realloc(node_lat, node_capacite * sizeof(double));
    node_lon = realloc(node_lon, node_capacite * sizeof(double));
    graphe = realloc(graphe, node_capacite * sizeof(Way*));

    if (!node_id || !node_lat || !node_lon || !graphe) {
        printf("Allocation memoire echouee\n");
        exit(1);
    }

    for (int i = graphe_size; i < node_capacite; i++){
        graphe[i] = NULL;
    }

    graphe_size = node_capacite;
}

//attribue une vitesse en fonction du type de route venant de osm
int vit_defaut(const char *highway) {
    if (!highway) return 50;

    if (strcmp(highway, "motorway") == 0) return 130;
    if (strcmp(highway, "trunk") == 0) return 110;
    if (strcmp(highway, "primary") == 0) return 90;
    if (strcmp(highway, "secondary") == 0) return 80;
    if (strcmp(highway, "tertiary") == 0) return 70;
    if (strcmp(highway, "residential") == 0) return 50;
    if (strcmp(highway, "service") == 0) return 30;

    return 50;
}

//obtenir l'id interne qui correspond a un id OSM
int getNodeId(long long id) {
    unsigned long h = hashFunction(id);

    HashNode *temp = tableHash[h];
    while (temp) {
        if (temp->osm_id == id){
            return temp->index;
        }
        temp = temp->suiv;
    }

    assure_node_capacite();

    int idx = node_count++;

    node_id[idx] = id;

    HashNode *n = malloc(sizeof(HashNode));
    if (!n) {
        printf("malloc failed\n");
        exit(1);
    }

    n->osm_id = id;
    n->index = idx;
    n->suiv = tableHash[h];
    tableHash[h] = n;

    return idx;
}

//conversion de degres en radians pour calcul
double deg_vers_rad(double deg) {
    return deg * 3.14159265358979323846 / 180.0;
}

//formule de haversine permettant de calculer la distance entre deux points coordonnées GPS
double haversine(double lat1, double lon1, double lat2, double lon2) {
    double dlat = deg_vers_rad(lat2 - lat1);
    double dlon = deg_vers_rad(lon2 - lon1);

    lat1 = deg_vers_rad(lat1);
    lat2 = deg_vers_rad(lat2);

    double a =
        sin(dlat / 2) * sin(dlat / 2) +
        cos(lat1) * cos(lat2) *
        sin(dlon / 2) * sin(dlon / 2);

    double c = 2 * atan2(sqrt(a), sqrt(1 - a));

    return RAYON_TERRE * c;
}

//ajouter une way dans notre graphe
void addWay(int from, int to, int vitesse, double dist) {
    Way *e = malloc(sizeof(Way));
    if (!e) {
        printf("malloc failed\n");
        exit(1);
    }

    e->to = to;
    e->distance = dist;
    e->vitesse = vitesse;

    if (vitesse > 0){
        e->temps = dist / (vitesse * 1000.0 / 3600.0);
    }
    else{
        e->temps = INFINITY;
    }
    e->suiv = graphe[from];
    graphe[from] = e;
}

//enregistre les coordonnées de chaque noeud (fonction readOSM)
int node_callback(const void *user_data, const readosm_node *node) {
    int idx = getNodeId(node->id);

    node_lat[idx] = node->latitude;
    node_lon[idx] = node->longitude;

    return READOSM_OK;
}

//convertit une valeur mph vers kmh
int parse_maxvitesse(const char *value) {
    if (!value) {return 0;}

    char *endptr;
    double v = strtod(value, &endptr);

    if (endptr != value) {
        if (strstr(value, "mph")){
            return (int)(v * 1.60934);
        }
        return (int)v;
    }

    return 0;
}

//fonction readOSM, appelée pour chaque route 
//parcourt les noeuds d'une route et créer les way entre chacun des noeuds, puis calcule
//la distance et le temps
int way_callback(const void *user_data, const readosm_way *way) {
    if (!way->node_refs) return READOSM_OK;

    int isRoad = 0;
    int oneway = 0;
    int vitesse = 0;
    const char *highway = NULL;

    for (int i = 0; i < way->tag_count; i++) {
        const char *k = way->tags[i].key;
        const char *v = way->tags[i].value;

        if (strcmp(k, "highway") == 0) {
            isRoad = 1;
            highway = v;
        }

        if (strcmp(k, "maxvitesse") == 0){
            vitesse = parse_maxvitesse(v);
        }

        if (strcmp(k, "oneway") == 0) {
            if (strcmp(v, "-1") == 0){
                oneway = -1;
            }
            else if (!strcmp(v, "yes") || !strcmp(v, "1") || !strcmp(v, "true")){
                oneway = 1;
            }
        }
    }

    if (!isRoad) {return READOSM_OK;}

    if (vitesse == 0){
        vitesse = vit_defaut(highway);
    }

    for (int i = 0; i < way->node_ref_count - 1; i++) {
        int n1 = getNodeId(way->node_refs[i]);
        int n2 = getNodeId(way->node_refs[i + 1]);

        double dist = haversine(
            node_lat[n1], node_lon[n1],
            node_lat[n2], node_lon[n2]
        );

        if (oneway == 1) {
            addWay(n1, n2, vitesse, dist);
        } else if (oneway == -1) {
            addWay(n2, n1, vitesse, dist);
        } else {
            addWay(n1, n2, vitesse, dist);
            addWay(n2, n1, vitesse, dist);
        }
    }

    return READOSM_OK;
}

//main, ouvre le fichier osm, construit le graphe, affiche les ways et libere la mémoire
int main() {
    const char *filename = "map3.osm";
    const void *handle;

    if (readosm_open(filename, &handle) != READOSM_OK) {
        printf("Cannot open file\n");
        return 1;
    }

    if (readosm_parse(handle, NULL,
                      node_callback,
                      way_callback,
                      NULL) != READOSM_OK) {
        printf("Parse error\n");
        readosm_close(handle);
        return 1;
    }

    readosm_close(handle);

    for (int i = 0; i < node_count; i++) {
        for (Way *e = graphe[i]; e; e = e->suiv) {
            printf("%lld -> %lld | %.1f m | %d km/h | %.1f s\n",
                   node_id[i],
                   node_id[e->to],
                   e->distance,
                   e->vitesse,
                   e->temps);
        }
    }

    for (int i = 0; i < node_count; i++) {
        Way *e = graphe[i];
        while (e) {
            Way *suiv = e->suiv;
            free(e);
            e = suiv;
        }
    }

    for (int i = 0; i < HASH_SIZE; i++) {
        HashNode *h = tableHash[i];
        while (h) {
            HashNode *suiv = h->suiv;
            free(h);
            h = suiv;
        }
    }

    free(node_id);
    free(node_lat);
    free(node_lon);
    free(graphe);

    return 0;
}