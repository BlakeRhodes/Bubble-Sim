#include <furi.h>
#include <gui/gui.h>
#include <input/input.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <storage/storage.h>
#include <toolbox/path.h>

#define SCREEN_W 128
#define SCREEN_H 64

// Config file in /ext/apps_data/<appid>/bubble.cfg
#define BUBBLE_CFG_PATH APP_DATA_PATH("bubble.cfg")

// --- Tunable configuration limits -----------------------------------------

static const int   BUBBLE_MAX_COUNT        = 64;
static const float BUBBLE_MIN_RADIUS       = 1.0f;
static const float BUBBLE_MAX_RADIUS       = 32.0f;
static const float BUBBLE_MIN_SPEED        = 0.25f;
static const float BUBBLE_MAX_SPEED        = 64.0f;
static const float BUBBLE_MIN_RESTITUTION  = 0.0f;
static const float BUBBLE_MAX_RESTITUTION  = 1.0f;
static const float BUBBLE_MIN_POP          = 0.0f;
static const float BUBBLE_MAX_POP          = 1.0f;


// --- Physics ----------------------------------------------------------------

typedef struct {
    float x;
    float y;
    float vx;
    float vy;
    float ax;
    float ay;
    float radius;
    float inv_mass;    // 0 => static
    float restitution; // 0..1
    int group;         // 0 = small, 1 = medium, 2 = big
    int spawn_cooldown; // frames to skip collisions after spawn/respawn
    float pop_chance;   // 0..1 chance to "pop" on collision
    bool popped;        // flagged for respawn after physics step
} PhysicsBody;

typedef struct {
    float min_x;
    float max_x;
    float min_y;
    float max_y;
} WorldBounds;

static float ph_len2(float x, float y) {
    return x * x + y * y;
}

static bool body_is_visible_vertical(const PhysicsBody* b, const WorldBounds* bounds) {
    if(!bounds) return true;
    float top = b->y - b->radius;
    float bottom = b->y + b->radius;
    return !(bottom < bounds->min_y || top > bounds->max_y);
}

// --- RNG helper -------------------------------------------------------------

typedef struct {
    uint32_t state;
} SimpleRng;

static void rng_init(SimpleRng* rng, uint32_t seed) {
    rng->state = seed ? seed : 1u;
}

static uint32_t rng_next(SimpleRng* rng) {
    rng->state = rng->state * 1664525u + 1013904223u;
    return rng->state;
}

static float rng_next_float01(SimpleRng* rng) {
    return (float)(rng_next(rng) & 0x00FFFFFFu) / (float)0x01000000u;
}

// Physics step now has access to RNG for pop chance
static void physics_step(
    PhysicsBody* bodies,
    size_t count,
    float dt,
    float gravity_y,
    const WorldBounds* bounds,
    SimpleRng* rng
) {
    if(dt <= 0.0f) return;
    if(!bodies || count == 0) return;

    // 1) Integrate velocities and positions
    for(size_t i = 0; i < count; i++) {
        PhysicsBody* b = &bodies[i];

        if(b->inv_mass > 0.0f && !b->popped) {
            // apply acceleration + gravity
            b->vy += (b->ay + gravity_y) * dt;
            b->vx += b->ax * dt;

            b->x += b->vx * dt;
            b->y += b->vy * dt;
        }

        // Wall collisions (horizontal only – let bubbles pass through top/bottom)
        if(bounds) {
            float r = b->radius;
            if(b->x - r < bounds->min_x) {
                b->x = bounds->min_x + r;
                if(b->vx < 0.0f) b->vx = -b->vx * b->restitution;
            } else if(b->x + r > bounds->max_x) {
                b->x = bounds->max_x - r;
                if(b->vx > 0.0f) b->vx = -b->vx * b->restitution;
            }
        }

        // Decrement spawn cooldown
        if(b->spawn_cooldown > 0) {
            b->spawn_cooldown--;
        }
    }

    // 2) Naive O(n^2) circle–circle collision resolution
    for(size_t i = 0; i < count; i++) {
        PhysicsBody* a = &bodies[i];
        if(a->popped) continue; // skip popped bodies

        bool vis_a = body_is_visible_vertical(a, bounds);

        for(size_t j = i + 1; j < count; j++) {
            PhysicsBody* b = &bodies[j];
            if(b->popped) continue; // skip popped bodies

            bool vis_b = body_is_visible_vertical(b, bounds);

            // Skip collisions when both are offscreen vertically
            if(!vis_a && !vis_b) continue;

            // Skip collisions if either body is in spawn cooldown
            if(a->spawn_cooldown > 0 || b->spawn_cooldown > 0) continue;

            float dx = b->x - a->x;
            float dy = b->y - a->y;
            float r_sum = a->radius + b->radius;
            float dist2 = ph_len2(dx, dy);

            if(dist2 <= 0.00001f) {
                // prevent NaNs – give them a tiny separation
                dx = 0.001f;
                dy = 0.0f;
                dist2 = ph_len2(dx, dy);
            }

            if(dist2 > r_sum * r_sum) continue; // no overlap

            float dist = sqrtf(dist2);
            float penetration = r_sum - dist;
            if(penetration <= 0.0f) continue;

            // Normal from a -> b
            float nx = dx / dist;
            float ny = dy / dist;

            float inv_ma = a->inv_mass;
            float inv_mb = b->inv_mass;
            float inv_sum = inv_ma + inv_mb;
            if(inv_sum <= 0.0f) {
                // both static
                continue;
            }

            // Positional correction proportional to inverse mass
            float move_a = (inv_ma / inv_sum) * penetration;
            float move_b = (inv_mb / inv_sum) * penetration;

            if(inv_ma > 0.0f) {
                a->x -= nx * move_a;
                a->y -= ny * move_a;
            }
            if(inv_mb > 0.0f) {
                b->x += nx * move_b;
                b->y += ny * move_b;
            }

            // Relative velocity along normal
            float rvx = b->vx - a->vx;
            float rvy = b->vy - a->vy;
            float vel_norm = rvx * nx + rvy * ny;

            // if separating, skip bounce
            if(vel_norm > 0.0f) continue;

            // Combine restitution
            float e = (a->restitution + b->restitution) * 0.5f;

            // Impulse scalar
            float j_impulse = -(1.0f + e) * vel_norm;
            j_impulse /= inv_sum;

            float ix = j_impulse * nx;
            float iy = j_impulse * ny;

            if(inv_ma > 0.0f) {
                a->vx -= ix * inv_ma;
                a->vy -= iy * inv_ma;
            }
            if(inv_mb > 0.0f) {
                b->vx += ix * inv_mb;
                b->vy += iy * inv_mb;
            }

            // POP logic: chance-based removal on collision
            if(rng) {
                float avg_pop = (a->pop_chance + b->pop_chance) * 0.5f;
                if(avg_pop > 0.0f && rng_next_float01(rng) < avg_pop) {
                    // Pop the smaller bubble (feels a bit more natural)
                    PhysicsBody* victim = (a->radius <= b->radius) ? a : b;
                    victim->popped = true;
                }
            }
        }
    }
}

// --- Bubble sim app ---------------------------------------------------------

#define MAX_BODIES 48
#define GROUP_COUNT 3
#define SPAWN_COOLDOWN_FRAMES 10

typedef struct {
    int count;          // number of bodies in this group
    float radius;       // visual + collision radius
    float rise_speed;   // base upward speed (negative vy, since y grows downward)
    float restitution;  // bounciness 0..1
    float pop_chance;   // chance to pop on collision
    const char* name;   // not stored on disk
} BubbleGroupConfig;

// What gets written to disk (no pointers)
typedef struct {
    int count;
    float radius;
    float rise_speed;
    float restitution;
    float pop_chance;
} BubbleGroupConfigDisk;

typedef struct {
    BubbleGroupConfigDisk groups[GROUP_COUNT];
} BubbleConfig;

typedef enum {
    ConfigFieldCount = 0,
    ConfigFieldRadius,
    ConfigFieldSpeed,
    ConfigFieldRestitution,
    ConfigFieldPopChance,
    ConfigFieldCountEnum,
} ConfigField;

typedef struct {
    Gui* gui;
    ViewPort* view_port;
    FuriMessageQueue* queue;

    PhysicsBody bodies[MAX_BODIES];
    size_t body_count;

    WorldBounds bounds;
    float gravity_y;

    BubbleGroupConfig groups[GROUP_COUNT];
    int selected_group;   // 0,1,2
    ConfigField menu_field;

    SimpleRng rng;

    bool hud_visible;     // NEW: toggles HUD (footer text + highlight)
} BubbleApp;

typedef enum {
    EventTypeInput,
} BubbleEventType;

typedef struct {
    BubbleEventType type;
    InputEvent input;
} BubbleEvent;

// --- Config save/load -------------------------------------------------------

static void bubble_save_config(const BubbleApp* app) {
    Storage* storage = furi_record_open(RECORD_STORAGE);
    if(!storage) return;

    // Ensure app data directory exists: /ext/apps_data/<appid>/
    storage_common_mkdir(storage, APP_DATA_PATH(""));

    File* file = storage_file_alloc(storage);
    if(!file) {
        furi_record_close(RECORD_STORAGE);
        return;
    }

    // storage_file_open returns bool: true on success
    if(storage_file_open(file, BUBBLE_CFG_PATH, FSAM_WRITE, FSOM_CREATE_ALWAYS)) {
        BubbleConfig cfg;

        for(int i = 0; i < GROUP_COUNT; i++) {
            cfg.groups[i].count = app->groups[i].count;
            cfg.groups[i].radius = app->groups[i].radius;
            cfg.groups[i].rise_speed = app->groups[i].rise_speed;
            cfg.groups[i].restitution = app->groups[i].restitution;
            cfg.groups[i].pop_chance = app->groups[i].pop_chance;
        }

        storage_file_write(file, &cfg, sizeof(cfg));
        storage_file_sync(file);
    }

    storage_file_close(file);
    storage_file_free(file);
    furi_record_close(RECORD_STORAGE);
}

static void bubble_load_config(BubbleApp* app) {
    Storage* storage = furi_record_open(RECORD_STORAGE);
    if(!storage) return;

    File* file = storage_file_alloc(storage);
    if(!file) {
        furi_record_close(RECORD_STORAGE);
        return;
    }

    if(storage_file_open(file, BUBBLE_CFG_PATH, FSAM_READ, FSOM_OPEN_EXISTING)) {
        BubbleConfig cfg;
        size_t rd = storage_file_read(file, &cfg, sizeof(cfg));
        if(rd == sizeof(cfg)) {
            // Copy into runtime groups, preserving .name pointers
            for(int i = 0; i < GROUP_COUNT; i++) {
                app->groups[i].count = cfg.groups[i].count;
                app->groups[i].radius = cfg.groups[i].radius;
                app->groups[i].rise_speed = cfg.groups[i].rise_speed;
                app->groups[i].restitution = cfg.groups[i].restitution;
                app->groups[i].pop_chance = cfg.groups[i].pop_chance;
            }
        }
    }

    storage_file_close(file);
    storage_file_free(file);
    furi_record_close(RECORD_STORAGE);
}

// --- Bubble sim helpers -----------------------------------------------------

static void bubble_app_init_groups(BubbleApp* app) {
    app->groups[0].name = "Small";
    app->groups[0].count = 22;
    app->groups[0].radius = 3.0f;
    app->groups[0].rise_speed = 60.0f;
    app->groups[0].restitution = 0.8f;
    app->groups[0].pop_chance = 1.0f; // default: no popping

    app->groups[1].name = "Medium";
    app->groups[1].count = 10;
    app->groups[1].radius = 8.0f;
    app->groups[1].rise_speed = 11.0f;
    app->groups[1].restitution = 0.15f;
    app->groups[1].pop_chance = 0.10f;

    app->groups[2].name = "Large";
    app->groups[2].count = 4;
    app->groups[2].radius = 16.0f;
    app->groups[2].rise_speed = 4.0f;
    app->groups[2].restitution = 0.05f;
    app->groups[2].pop_chance = 0.10f;
}

// Rebuild all bodies based on group configs
static void bubble_app_build_bodies(BubbleApp* app) {
    app->body_count = 0;

    for(int g = 0; g < GROUP_COUNT; g++) {
        BubbleGroupConfig* cfg = &app->groups[g];
        int count = cfg->count;
        if(count < 0) count = 0;

        for(int i = 0; i < count && app->body_count < MAX_BODIES; i++) {
            PhysicsBody* b = &app->bodies[app->body_count++];

            b->radius = cfg->radius;
            b->inv_mass = 1.0f; // all dynamic
            b->restitution = cfg->restitution;
            b->group = g;
            b->pop_chance = cfg->pop_chance;
            b->popped = false;

            float r = b->radius;

            // random horizontal position
            float x = (float)(app->bounds.min_x + r) +
                      rng_next_float01(&app->rng) *
                          (float)((app->bounds.max_x - r) - (app->bounds.min_x + r));

            // spawn well below the bottom to avoid visible jitter
            float y_base = app->bounds.max_y + r + 40.0f;
            float y = y_base + rng_next_float01(&app->rng) * 20.0f;

            b->x = x;
            b->y = y;

            // Upward velocity (negative in screen coords)
            float jitter = (rng_next_float01(&app->rng) - 0.5f) * cfg->rise_speed * 0.2f;
            b->vx = jitter;
            b->vy = -cfg->rise_speed;

            b->ax = 0.0f;
            b->ay = 0.0f;
            b->spawn_cooldown = SPAWN_COOLDOWN_FRAMES;
        }
    }
}

// Reinitialize only a single group's bodies
static void bubble_app_reinit_group(BubbleApp* app, int group_id) {
    if(group_id < 0 || group_id >= GROUP_COUNT) return;

    BubbleGroupConfig* cfg = &app->groups[group_id];

    // First, remove existing bodies of this group
    size_t write = 0;
    for(size_t i = 0; i < app->body_count; i++) {
        if(app->bodies[i].group == group_id) continue;
        if(write != i) app->bodies[write] = app->bodies[i];
        write++;
    }
    app->body_count = write;

    // Add new ones based on updated config
    int count = cfg->count;
    if(count < 0) count = 0;

    for(int i = 0; i < count && app->body_count < MAX_BODIES; i++) {
        PhysicsBody* b = &app->bodies[app->body_count++];

        b->radius = cfg->radius;
        b->inv_mass = 1.0f;
        b->restitution = cfg->restitution;
        b->group = group_id;
        b->pop_chance = cfg->pop_chance;
        b->popped = false;

        float r = b->radius;

        float x = (float)(app->bounds.min_x + r) +
                  rng_next_float01(&app->rng) *
                      (float)((app->bounds.max_x - r) - (app->bounds.min_x + r));

        float y_base = app->bounds.max_y + r + 40.0f;
        float y = y_base + rng_next_float01(&app->rng) * 20.0f;

        b->x = x;
        b->y = y;

        float jitter = (rng_next_float01(&app->rng) - 0.5f) * cfg->rise_speed * 0.2f;
        b->vx = jitter;
        b->vy = -cfg->rise_speed;

        b->ax = 0.0f;
        b->ay = 0.0f;
        b->spawn_cooldown = SPAWN_COOLDOWN_FRAMES;
    }
}

// Respawn a single bubble well below the screen
static void bubble_respawn_body(BubbleApp* app, PhysicsBody* b) {
    BubbleGroupConfig* cfg = &app->groups[b->group];

    float r = b->radius;

    float x = (float)(app->bounds.min_x + r) +
              rng_next_float01(&app->rng) *
                  (float)((app->bounds.max_x - r) - (app->bounds.min_x + r));

    float y_base = app->bounds.max_y + r + 40.0f;
    float y = y_base + rng_next_float01(&app->rng) * 20.0f;

    b->x = x;
    b->y = y;

    float jitter = (rng_next_float01(&app->rng) - 0.5f) * cfg->rise_speed * 0.2f;
    b->vx = jitter;
    b->vy = -cfg->rise_speed;

    b->ax = 0.0f;
    b->ay = 0.0f;
    b->spawn_cooldown = SPAWN_COOLDOWN_FRAMES;
    b->popped = false;
}

// --- Drawing ----------------------------------------------------------------

static void bubble_draw_body(Canvas* canvas, const PhysicsBody* b, bool selected) {
    int x = (int)(b->x + 0.5f);
    int y = (int)(b->y + 0.5f);
    int r = (int)(b->radius + 0.5f);
    if(r < 1) r = 1;

    if(x + r < 0 || x - r >= SCREEN_W) return;
    if(y + r < 0 || y - r >= SCREEN_H) return;

    // Selected group gets a thicker border: draw 2 concentric circles
    if(selected) {
        canvas_draw_circle(canvas, x, y, r);
        if(r > 1) {
            canvas_draw_circle(canvas, x, y, r - 1);
        }
    } else {
        canvas_draw_circle(canvas, x, y, r);
    }
}

static void bubble_draw(Canvas* canvas, void* ctx) {
    BubbleApp* app = ctx;
    canvas_clear(canvas);

    // Draw bodies only
    for(size_t i = 0; i < app->body_count; i++) {
        const PhysicsBody* b = &app->bodies[i];
        // When HUD is hidden, we don't visually highlight the selected group
        bool selected = app->hud_visible && (b->group == app->selected_group);
        bubble_draw_body(canvas, b, selected);
    }

    // Footer: show which field is being edited + value (only if HUD visible)
    if(app->hud_visible) {
        BubbleGroupConfig* cfg = &app->groups[app->selected_group];

        canvas_set_font(canvas, FontSecondary);
        char buf[32];

        switch(app->menu_field) {
            case ConfigFieldCount:
                snprintf(buf, sizeof(buf), "Count=%d", cfg->count);
                break;
            case ConfigFieldRadius:
                snprintf(buf, sizeof(buf), "Radius=%.1f", (double)cfg->radius);
                break;
            case ConfigFieldSpeed:
                snprintf(buf, sizeof(buf), "Speed=%.2f", (double)cfg->rise_speed);
                break;
            case ConfigFieldRestitution:
                int res = (int)(cfg->restitution * 100.0f + 0.5f); // round to nearest int
                snprintf(buf, sizeof(buf), "Bounce=%d%%", res);
                break;
            case ConfigFieldPopChance:
                int pct = (int)(cfg->pop_chance * 100.0f + 0.5f); // round to nearest int
                snprintf(buf, sizeof(buf), "Pop=%d%%", pct);
		break;
            default:
                snprintf(buf, sizeof(buf), "?");
                break;
        }

        // bottom line: y = SCREEN_H - 1
        canvas_draw_str(canvas, 0, SCREEN_H - 1, buf);
    }
}

// --- Input handling ---------------------------------------------------------

static void bubble_input_cb(InputEvent* input, void* ctx) {
    BubbleApp* app = ctx;
    BubbleEvent ev = {.type = EventTypeInput, .input = *input};
    furi_message_queue_put(app->queue, &ev, 0);
}

static void bubble_save_and_reinit(BubbleApp* app) {
    bubble_app_reinit_group(app, app->selected_group);
    bubble_save_config(app);
}

static void bubble_adjust_field(BubbleApp* app, int dir) {
    BubbleGroupConfig* cfg = &app->groups[app->selected_group];

    switch(app->menu_field) {
        case ConfigFieldCount:
            cfg->count += dir;
            if(cfg->count < 0) cfg->count = 0;
            if(cfg->count > BUBBLE_MAX_COUNT) cfg->count = BUBBLE_MAX_COUNT;
            bubble_save_and_reinit(app);
            break;

        case ConfigFieldRadius:
            cfg->radius += (float)dir * 0.25f;
            if(cfg->radius < BUBBLE_MIN_RADIUS) cfg->radius = BUBBLE_MIN_RADIUS;
            if(cfg->radius > BUBBLE_MAX_RADIUS) cfg->radius = BUBBLE_MAX_RADIUS;
            bubble_save_and_reinit(app);
            break;

        case ConfigFieldSpeed:
            cfg->rise_speed += (float)dir * 1.0f;
            if(cfg->rise_speed < BUBBLE_MIN_SPEED) cfg->rise_speed = BUBBLE_MIN_SPEED;
            if(cfg->rise_speed > BUBBLE_MAX_SPEED) cfg->rise_speed = BUBBLE_MAX_SPEED;
            bubble_save_and_reinit(app);
            break;

        case ConfigFieldRestitution:
            cfg->restitution += (float)dir * 0.01f;
            if(cfg->restitution < BUBBLE_MIN_RESTITUTION) cfg->restitution = BUBBLE_MIN_RESTITUTION;
            if(cfg->restitution > BUBBLE_MAX_RESTITUTION) cfg->restitution = BUBBLE_MAX_RESTITUTION;
            bubble_save_and_reinit(app);
            break;

        case ConfigFieldPopChance:
            cfg->pop_chance += (float)dir * 0.01f;
            if(cfg->pop_chance < BUBBLE_MIN_POP) cfg->pop_chance = BUBBLE_MIN_POP;
            if(cfg->pop_chance > BUBBLE_MAX_POP) cfg->pop_chance = BUBBLE_MAX_POP;
            bubble_save_and_reinit(app);
            break;

        default:
            break;
    }
}


static void bubble_handle_input(BubbleApp* app, InputEvent* in, bool* running) {
    // First, handle long-press OK to toggle HUD visibility
    if((in->type == InputTypeLong) && (in->key == InputKeyOk)) {
        app->hud_visible = !app->hud_visible;
        return;
    }

    // For everything else, we only care about short/repeat events
    if(!(in->type == InputTypeShort || in->type == InputTypeRepeat)) return;

    switch(in->key) {
        case InputKeyBack:
            // Back now exits immediately
            *running = false;
            return;

        case InputKeyUp:
            // Change which property is selected
            if(app->menu_field == 0) {
                app->menu_field = (ConfigField)(ConfigFieldCountEnum - 1);
            } else {
                app->menu_field = (ConfigField)(app->menu_field - 1);
            }
            break;

        case InputKeyDown:
            // Change which property is selected
            app->menu_field = (ConfigField)(app->menu_field + 1);
            if(app->menu_field >= ConfigFieldCountEnum) {
                app->menu_field = ConfigFieldCount;
            }
            break;

        case InputKeyLeft:
            // Decrease value of current property
            bubble_adjust_field(app, -1);
            break;

        case InputKeyRight:
            // Increase value of current property
            bubble_adjust_field(app, +1);
            break;

        case InputKeyOk:
            // Cycle group (Small -> Medium -> Large -> Small ...)
            app->selected_group++;
            if(app->selected_group >= GROUP_COUNT) {
                app->selected_group = 0;
            }
            break;

        default:
            break;
    }
}

// --- Entry ------------------------------------------------------------------

int32_t bubble_sim_app(void* p) {
    UNUSED(p);

    BubbleApp* app = malloc(sizeof(BubbleApp));
    furi_check(app);
    memset(app, 0, sizeof(BubbleApp));

    // Init RNG
    rng_init(&app->rng, furi_get_tick());

    // World bounds (screen interior)
    app->bounds.min_x = 0.0f;
    app->bounds.max_x = (float)(SCREEN_W - 1);
    app->bounds.min_y = 0.0f;
    app->bounds.max_y = (float)(SCREEN_H - 1);

    app->gravity_y = 0.0f; // no gravity; bubbles just rise by initial velocity

    // Defaults, then load from disk if present
    bubble_app_init_groups(app);
    bubble_load_config(app);

    app->selected_group = 0;
    app->menu_field = ConfigFieldCount;
    app->hud_visible = true; // HUD visible by default

    bubble_app_build_bodies(app);

    // Flipper GUI plumbing
    app->gui = furi_record_open(RECORD_GUI);
    furi_check(app->gui);

    app->view_port = view_port_alloc();
    furi_check(app->view_port);

    app->queue = furi_message_queue_alloc(8, sizeof(BubbleEvent));
    furi_check(app->queue);

    view_port_draw_callback_set(app->view_port, bubble_draw, app);
    view_port_input_callback_set(app->view_port, bubble_input_cb, app);
    gui_add_view_port(app->gui, app->view_port, GuiLayerFullscreen);

    bool running = true;
    BubbleEvent ev;

    while(running) {
        // Handle inputs
        if(furi_message_queue_get(app->queue, &ev, 0) == FuriStatusOk) {
            bubble_handle_input(app, &ev.input, &running);
        }

        // Physics step
        const float dt = 0.03f; // ~30 ms
        physics_step(app->bodies, app->body_count, dt, app->gravity_y, &app->bounds, &app->rng);

        // Handle popped bubbles: respawn them
        for(size_t i = 0; i < app->body_count; i++) {
            PhysicsBody* b = &app->bodies[i];
            if(b->popped) {
                bubble_respawn_body(app, b);
            }
        }

        // If a bubble floats off the top, respawn well below the screen
        for(size_t i = 0; i < app->body_count; i++) {
            PhysicsBody* b = &app->bodies[i];
            if(b->y + b->radius < app->bounds.min_y - 20.0f) {
                bubble_respawn_body(app, b);
            }
        }

        view_port_update(app->view_port);
        furi_delay_ms(30);
    }

    gui_remove_view_port(app->gui, app->view_port);
    view_port_free(app->view_port);
    furi_record_close(RECORD_GUI);

    furi_message_queue_free(app->queue);
    free(app);

    return 0;
}

