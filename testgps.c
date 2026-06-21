#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <readosm.h>
#include <raylib.h>

#define RAYON_TERRE 6371000.0
#define HASH_SIZE 200003  

//raylib var
float zoom = 1.0f;
float offsetX = 0;
float offsetY = 0;

int dragging = 0;
Vector2 lastMouse;

typedef struct {
    float x, y;
} Vec2;

Vec2 *screen_pos = NULL;
float scale = 1.0f;
float offset_x = 0;
float offset_y = 0;

Vec2 project(double lat, double lon,
             double min_lat, double min_lon)
{
    Vec2 v;

    v.x = (lon - min_lon) * 100000.0f;
    v.y = (lat - min_lat) * 100000.0f;

    return v;
}

//Path pour le dijkstra
typedef struct {
    int *nodes;
    int count;
    double total_time;
    double total_distance;
} Path;

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
char **node_name = NULL;
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
    node_name = realloc(node_name, node_capacite * sizeof(char*));

    if (!node_id || !node_lat || !node_lon || !graphe || !node_name) {
        printf("Allocation memoire echouee\n");
        exit(1);
    }

    for (int i = graphe_size; i < node_capacite; i++){
        graphe[i] = NULL;
        node_name[i] = NULL;
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

//enregistre les coordonnées de chaque noeud (fonction readOSM)
int node_callback(const void *user_data, const readosm_node *node)
{
    int idx = getNodeId(node->id);

    node_lat[idx] = node->latitude;
    node_lon[idx] = node->longitude;

    for (int i = 0; i < node->tag_count; i++) {
        if (strcmp(node->tags[i].key, "name") == 0) {

            node_name[idx] =
                strdup(node->tags[i].value);

            break;
        }
    }

    return READOSM_OK;
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



//struct pour la distance vers une node
typedef struct {
    int node;
    double dist;
} HeapNode;

typedef struct {
    HeapNode *data;
    int taille;
    int capacite;
} MinHeap;

//creation d'une heap
MinHeap *creeHeap(int capacite) {
    MinHeap *h = malloc(sizeof(MinHeap));

    h->data = malloc(capacite * sizeof(HeapNode));
    h->taille = 0;
    h->capacite = capacite;

    return h;
}

//interchange deux heapnode
void swapHeap(HeapNode *a, HeapNode *b) {
    HeapNode temp = *a;
    *a = *b;
    *b = temp;
}

//ajoute une node, si l'heap n'est pas assez grande on realloc
void addHeap(MinHeap *h, int node, double dist) {
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

        swapHeap(&h->data[parent], &h->data[i]);

        i = parent;
    }
}

//enleve le plus petit element et restore la heap
HeapNode removeHeap(MinHeap *h) {
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

        swapHeap(&h->data[i], &h->data[smallest]);

        i = smallest;
    }

    return min;
}

int videHeap(MinHeap *h) {
    return h->taille == 0;
}

void freeHeap(MinHeap *h) {
    free(h->data);
    free(h);
}

//trouve le chemin le plus court via dijkstra
Path *dijkstra(int source, int destination)
{
    double *dist = malloc(node_count * sizeof(double));
    int *pred = malloc(node_count * sizeof(int));

    if (!dist || !pred) {
        printf("Allocation memoire echouee\n");
        return NULL;
    }

    for (int i = 0; i < node_count; i++) {
        dist[i] = INFINITY;
        pred[i] = -1;
    }

    dist[source] = 0.0;

    MinHeap *heap = creeHeap(1024);

    addHeap(heap, source, 0.0);

    while (!videHeap(heap)) {

        HeapNode courant = removeHeap(heap);

        int u = courant.node;

        if (courant.dist > dist[u])
            continue;

        if (u == destination)
            break;

        for (Way *e = graphe[u]; e; e = e->suiv) {

            int v = e->to;

            double alt = dist[u] + e->temps;
            /* double alt = dist[u] + e->distance; */

            if (alt < dist[v]) {

                dist[v] = alt;
                pred[v] = u;

                addHeap(heap, v, alt);
            }
        }
    }

    //si destination jms atteinte
    if (dist[destination] == INFINITY) {

        printf("Aucun chemin trouve\n");

        freeHeap(heap);
        free(dist);
        free(pred);

        return NULL;
    }

    int *path = malloc(node_count * sizeof(int));

    if (!path) {
        freeHeap(heap);
        free(dist);
        free(pred);
        return NULL;
    }

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
        int to = path[i - 1];

        for (Way *e = graphe[from]; e; e = e->suiv) {

            if (e->to == to) {
                total_distance += e->distance;
                break;
            }
        }
    }

    printf("\nRESULTAT \n");
    printf("Temps minimal : %.1f s (%.1f min)\n",
           dist[destination],
           dist[destination] / 60.0);

    printf("Distance totale : %.2f km\n",
           total_distance / 1000.0);

    printf("Nombre de noeuds : %d\n", n);

    printf("\nSEGMENTS \n");

    for (int i = n - 1; i > 0; i--) {

        int from = path[i];
        int to = path[i - 1];

        for (Way *e = graphe[from]; e; e = e->suiv) {

            if (e->to == to) {

                const char *from_name =
                    (node_name[from] && strlen(node_name[from]) > 0)
                        ? node_name[from]
                        : "Unnamed";

                const char *to_name =
                    (node_name[to] && strlen(node_name[to]) > 0)
                        ? node_name[to]
                        : "Unnamed";
                printf(
                    "%s (%lld) -> %s (%lld) | %.0f m | %d km/h | %.1f s\n",
                    from_name,
                    node_id[from],
                    to_name,
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

    //construit le chemin pour pouvoir le retourner
    Path *result = malloc(sizeof(Path));

    if (!result) {
        free(path);
        freeHeap(heap);
        free(dist);
        free(pred);
        return NULL;
    }

    result->nodes = malloc(n * sizeof(int));

    if (!result->nodes) {
        free(result);
        free(path);
        freeHeap(heap);
        free(dist);
        free(pred);
        return NULL;
    }

    for (int i = 0; i < n; i++) {
        result->nodes[i] = path[i];
    }

    result->count = n;
    result->total_time = dist[destination];
    result->total_distance = total_distance;

    free(path);

    freeHeap(heap);
    free(dist);
    free(pred);

    return result;
}

void free_path(Path *p)
{
    if (!p)
        return;

    free(p->nodes);
    free(p);
}

typedef struct {
    int from;
    int to;
} TempEdge;

TempEdge connect_to_nearest_road(int node)
{
    TempEdge temp = {-1, -1};

    if (graphe[node] != NULL)
        return temp;

    double best = INFINITY;
    int nearest = -1;

    for (int i = 0; i < node_count; i++) {

        if (graphe[i] == NULL)
            continue;

        double d = haversine(
            node_lat[node],
            node_lon[node],
            node_lat[i],
            node_lon[i]
        );

        if (d < best) {
            best = d;
            nearest = i;
        }
    }

    if (nearest != -1) {

        addWay(node, nearest, 5, best);
        addWay(nearest, node, 5, best);

        temp.from = node;
        temp.to = nearest;
    }

    return temp;
}

void remove_last_edge(int from)
{
    if (!graphe[from])
        return;

    Way *tmp = graphe[from];
    graphe[from] = tmp->suiv;
    free(tmp);
}

/*int trouverNode(long long osm_id) {

    unsigned long h = hashFunction(osm_id);

    HashNode *cur = tableHash[h];

    while (cur) {

        if (cur->osm_id == osm_id)
            return cur->index;

        cur = cur->suiv;
    }

    return -1;
}*/

int start_node = -1;
int end_node = -1;
float pick_radius = 8.0f; // in screen pixels
int recompute_path = 0;

//main, ouvre le fichier osm, construit le graphe, affiche les ways et libere la mémoire
int main() {
    const char *filename = "map6.osm";
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

    //non necessaire grâce à raylib mais permet d'utiliser dijkstra en donnant nous meme les nodes
    /*long long depart_osm = 13789018119;
    long long arrivee_osm = 11558432440;

    int depart = trouverNode(depart_osm);
    int arrivee = trouverNode(arrivee_osm);*:

    /*if (depart != -1 && arrivee != -1) {
        Path *path = dijkstra(depart, arrivee);

        if (path) {
            printf("Path contient %d nodes\n", path->count);

            free_path(path);
        }
    }

    for (int i = 0; i < node_count; i++) {
        for (Way *e = graphe[i]; e; e = e->suiv) {
            printf("%lld -> %lld | %.1f m | %d km/h | %.1f s\n",
                   node_id[i],
                   node_id[e->to],
                   e->distance,
                   e->vitesse,
                   e->temps);
        }
    }*/

    //code pour raylib
    double min_lat = 1e9, min_lon = 1e9;
    double max_lat = -1e9, max_lon = -1e9;

    // calcule la taille de la fenetre
    for (int i = 0; i < node_count; i++) {
        if (node_lat[i] < min_lat) min_lat = node_lat[i];
        if (node_lon[i] < min_lon) min_lon = node_lon[i];
        if (node_lat[i] > max_lat) max_lat = node_lat[i];
        if (node_lon[i] > max_lon) max_lon = node_lon[i];
    }

    //position de l'ecran
    screen_pos = malloc(node_count * sizeof(Vec2));

    //resolution
    float scale_x = 1200.0f / (max_lon - min_lon);
    float scale_y = 800.0f  / (max_lat - min_lat);
    float scale = (scale_x < scale_y) ? scale_x : scale_y;

    //offset
    for (int i = 0; i < node_count; i++) {
        screen_pos[i].x = (node_lon[i] - min_lon) * scale;
        screen_pos[i].y = (max_lat - node_lat[i]) * scale; 
    }

    InitWindow(1200, 800, "OSM Dijkstra");
    SetTargetFPS(30);

    int depart = 1;
    int arrivee=1;
    Path *path = dijkstra(depart, arrivee);

    while (!WindowShouldClose()) {

        BeginDrawing();
        ClearBackground(RAYWHITE);

        Vector2 mouse = GetMousePosition();

        float mouseX = (mouse.x - offsetX) / zoom;
        float mouseY = (mouse.y - offsetY) / zoom;
        
        int hovered = -1;
        float best = 1000000.0f;

        for (int i = 0; i < node_count; i++) {

            float dx = screen_pos[i].x - mouseX;
            float dy = screen_pos[i].y - mouseY;

            float d2 = dx*dx + dy*dy;

            if (d2 < best && d2 < 100.0f) {
                best = d2;
                hovered = i;
            }
        }

        //clic gauche pour choisir node de depart
        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON) && !IsKeyDown(KEY_SPACE)) {

            float bestDist = 1e9;
            int bestNode = -1;

            for (int i = 0; i < node_count; i++) {

                float dx = screen_pos[i].x - mouseX;
                float dy = screen_pos[i].y - mouseY;

                float d2 = dx*dx + dy*dy;

                if (d2 < bestDist && d2 < (pick_radius * pick_radius)) {
                    bestDist = d2;
                    bestNode = i;
                }
            }

            if (bestNode != -1) {
                start_node = bestNode;
                recompute_path = 1;
            }
        }

        //clic droit pour node d'arrivee
        if (IsMouseButtonPressed(MOUSE_RIGHT_BUTTON)) {

            float bestDist = 1e9;
            int bestNode = -1;

            for (int i = 0; i < node_count; i++) {

                float dx = screen_pos[i].x - mouseX;
                float dy = screen_pos[i].y - mouseY;

                float d2 = dx*dx + dy*dy;

                if (d2 < bestDist && d2 < (pick_radius * pick_radius)) {
                    bestDist = d2;
                    bestNode = i;
                }
            }

            if (bestNode != -1) {
                end_node = bestNode;
                recompute_path = 1;
            }
        }

        if (start_node != -1 && end_node != -1 && recompute_path) {
            free_path(path);
            
            TempEdge s = connect_to_nearest_road(start_node);
            TempEdge e = connect_to_nearest_road(end_node);

            path = dijkstra(start_node, end_node);

            if (s.from != -1) {
                remove_last_edge(s.from);
                remove_last_edge(s.to);
            }

            if (e.from != -1) {
                remove_last_edge(e.from);
                remove_last_edge(e.to);
            }
            recompute_path = 0;
        }

        //clic gauche + espace pour bouger
        if (IsKeyDown(KEY_SPACE) && IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
            lastMouse = mouse;
        }

        dragging = IsKeyDown(KEY_SPACE) && IsMouseButtonDown(MOUSE_LEFT_BUTTON);

        if (dragging) {
            Vector2 delta = {
                mouse.x - lastMouse.x,
                mouse.y - lastMouse.y
            };

            offsetX += delta.x;
            offsetY += delta.y;

            lastMouse = mouse;
        }

        //gerer le zoom
        float wheel = GetMouseWheelMove();

        if (wheel != 0) {
            float mouseX = GetMouseX();
            float mouseY = GetMouseY();

            // coordonnees avant le zoom
            float worldX = (mouseX - offsetX) / zoom;
            float worldY = (mouseY - offsetY) / zoom;

            zoom *= (1.0f + wheel * 0.1f);

            if (zoom < 0.1f) zoom = 0.1f;
            if (zoom > 20.0f) zoom = 20.0f;

            //zoom sera centre sur le curseur
            offsetX = mouseX - worldX * zoom;
            offsetY = mouseY - worldY * zoom;
        }

        //dessine les routes
        for (int i = 0; i < node_count; i++) {
            for (Way *e = graphe[i]; e; e = e->suiv) {
                Vector2 a = {
                    screen_pos[i].x * zoom + offsetX,
                    screen_pos[i].y * zoom + offsetY
                };

                Vector2 b = {
                    screen_pos[e->to].x * zoom + offsetX,
                    screen_pos[e->to].y * zoom + offsetY
                };

                DrawLineV(a, b, LIGHTGRAY);
            }
        }

        //dessine le path du dijkstra
        if (path) {
            for (int i = 0; i < path->count - 1; i++) {
                int a = path->nodes[i];
                int b = path->nodes[i + 1];

                float ax = screen_pos[a].x * zoom + offsetX;
                float ay = screen_pos[a].y * zoom + offsetY;

                float bx = screen_pos[b].x * zoom + offsetX;
                float by = screen_pos[b].y * zoom + offsetY;

                DrawLine(
                    ax, ay, bx, by,
                    RED
                );
            }
        }

        //dessine toutes les nodes
        for (int i = 0; i < node_count; i++) {
            float sx = screen_pos[i].x * zoom + offsetX;
            float sy = screen_pos[i].y * zoom + offsetY;

            DrawCircle(sx, sy, 2, DARKBLUE);
        }

        if (hovered != -1) {

            char buffer[256];

            if (node_name[hovered]) {
                snprintf(buffer, sizeof(buffer),
                        "%s", node_name[hovered]);
            } else {
                snprintf(buffer, sizeof(buffer),
                        "OSM ID: %lld",
                        node_id[hovered]);
            }

            DrawRectangle(
                GetMouseX() + 10,
                GetMouseY() + 10,
                MeasureText(buffer, 20) + 10,
                30,
                Fade(WHITE, 0.9f)
            );

            DrawText(
                buffer,
                GetMouseX() + 15,
                GetMouseY() + 15,
                20,
                BLACK
            );
        }

        //dessine le point de depart
        if (start_node != -1) {
            DrawCircle(
                screen_pos[start_node].x * zoom + offsetX,
                screen_pos[start_node].y * zoom + offsetY,
                6, GREEN
            );
        }
        //et le point d'arrivee
        if (end_node != -1) {
            DrawCircle(
                screen_pos[end_node].x * zoom + offsetX,
                screen_pos[end_node].y * zoom + offsetY,
                6, RED
            );
        }
        EndDrawing();
    }

    CloseWindow();

    //libere l'espace
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