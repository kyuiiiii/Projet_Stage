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
    float distance;
    int vitesse;
    float temps; //temps théorique pour parcourir l'arête
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
int parse_maxspeed(const char *value) {
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

        if (strcmp(k, "maxspeed") == 0){
            vitesse = parse_maxspeed(v);
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

typedef struct {
    int node;
    double dist;
} HeapNode;

typedef struct {
    HeapNode *data;
    int taille;
    int capacite;
} MinHeap;

MinHeap *heap_create(int capacite) {
    MinHeap *h = malloc(sizeof(MinHeap));

    h->data = malloc(capacite * sizeof(HeapNode));
    h->taille = 0;
    h->capacite = capacite;

    return h;
}

void heap_swap(HeapNode *a, HeapNode *b) {
    HeapNode temp = *a;
    *a = *b;
    *b = temp;
}

void heap_push(MinHeap *h, int node, double dist) {
    if (h->taille >= h->capacite) {
        h->capacite *= 2;
        h->data = realloc(
            h->data,
            h->capacite * sizeof(HeapNode)
        );
    }

    int i = h->taille++;

    h->data[i].node = node;
    h->data[i].dist = dist;

    while (i > 0) {
        int parent = (i - 1) / 2;

        if (h->data[parent].dist <= h->data[i].dist)
            break;

        heap_swap(&h->data[parent], &h->data[i]);

        i = parent;
    }
}

HeapNode heap_pop(MinHeap *h) {
    HeapNode min = h->data[0];

    h->data[0] = h->data[--h->taille];

    int i = 0;

    while (1) {
        int left = 2 * i + 1;
        int right = 2 * i + 2;
        int smallest = i;

        if (left < h->taille &&
            h->data[left].dist < h->data[smallest].dist)
            smallest = left;

        if (right < h->taille &&
            h->data[right].dist < h->data[smallest].dist)
            smallest = right;

        if (smallest == i)
            break;

        heap_swap(&h->data[i], &h->data[smallest]);

        i = smallest;
    }

    return min;
}

int heap_empty(MinHeap *h) {
    return h->taille == 0;
}

void heap_free(MinHeap *h) {
    free(h->data);
    free(h);
}

void dijkstra(int source, int destination) {

    double *dist = malloc(node_count * sizeof(double));
    int *pred = malloc(node_count * sizeof(int));

    for (int i = 0; i < node_count; i++) {
        dist[i] = INFINITY;
        pred[i] = -1;
    }

    dist[source] = 0.0;

    MinHeap *heap = heap_create(1024);

    heap_push(heap, source, 0.0);

    while (!heap_empty(heap)) {

        HeapNode current = heap_pop(heap);

        int u = current.node;

        if (current.dist > dist[u])
            continue;

        if (u == destination)
            break;

        for (Way *e = graphe[u]; e; e = e->suiv) {

            int v = e->to;

            double alt = dist[u] + e->temps;
            //double alt = dist[u] + e->distance; //chemin plus court en metres plutot que temps

            if (alt < dist[v]) {

                dist[v] = alt;
                pred[v] = u;

                heap_push(heap, v, alt);
            }
        }
    }

    if (dist[destination] == INFINITY) {
        printf("Aucun chemin trouve\n");
    }
    else {

        int *path = malloc(node_count * sizeof(int));
        int n = 0;

        for (int v = destination;
            v != -1;
            v = pred[v])
        {
            path[n++] = v;
        }

        double total_distance = 0.0;

        for (int i = n - 1; i > 0; i--) {

            int from = path[i];
            int to   = path[i - 1];

            for (Way *e = graphe[from]; e; e = e->suiv) {

                if (e->to == to) {
                    total_distance += e->distance;
                    break;
                }
            }
        }

        printf("\n===== RESULTAT =====\n");
        printf("Temps minimal : %.1f s (%.1f min)\n",
            dist[destination],
            dist[destination] / 60.0);

        printf("Distance totale : %.2f km\n",
            total_distance / 1000.0);

        printf("Nombre de noeuds : %d\n", n);

        printf("\n===== SEGMENTS =====\n");

        for (int i = n - 1; i > 0; i--) {

            int from = path[i];
            int to   = path[i - 1];

            for (Way *e = graphe[from]; e; e = e->suiv) {

                if (e->to == to) {

                    printf(
                        "%lld -> %lld | %.0f m | %d km/h | %.1f s\n",
                        node_id[from],
                        node_id[to],
                        e->distance,
                        e->vitesse,
                        e->temps
                    );

                    break;
                }
            }
        }
        printf("\n");

        free(path);
    }

    heap_free(heap);
    free(dist);
    free(pred);
}

int findNode(long long osm_id) {

    unsigned long h = hashFunction(osm_id);

    HashNode *cur = tableHash[h];

    while (cur) {

        if (cur->osm_id == osm_id)
            return cur->index;

        cur = cur->suiv;
    }

    return -1;
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

    long long depart_osm = 13789018119;
    long long arrivee_osm = 11558432440;

    int depart = findNode(depart_osm);
    int arrivee = findNode(arrivee_osm);

    if (depart != -1 && arrivee != -1) {
        dijkstra(depart, arrivee);
    }

    /*for (int i = 0; i < node_count; i++) {
        for (Way *e = graphe[i]; e; e = e->suiv) {
            printf("%lld -> %lld | %.1f m | %d km/h | %.1f s\n",
                   node_id[i],
                   node_id[e->to],
                   e->distance,
                   e->vitesse,
                   e->temps);
        }
    }*/

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