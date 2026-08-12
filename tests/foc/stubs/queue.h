#ifndef TEST_QUEUE_H
#define TEST_QUEUE_H

#include "FreeRTOS.h"
#include <stddef.h>
#include <string.h>

struct StaticQueue_t
{
    uint8_t *storage = nullptr;
    size_t item_size = 0U;
    bool present = false;
};

typedef StaticQueue_t *QueueHandle_t;

inline QueueHandle_t xQueueCreateStatic(UBaseType_t length,
    UBaseType_t item_size,
    uint8_t *storage,
    StaticQueue_t *control)
{
    if(length != 1U || item_size == 0U || !storage || !control)
    {
        return nullptr;
    }

    control->storage = storage;
    control->item_size = item_size;
    control->present = false;
    return control;
}

inline BaseType_t xQueueOverwrite(QueueHandle_t queue, const void *item)
{
    if(!queue || !item){return pdFALSE;}
    memcpy(queue->storage, item, queue->item_size);
    queue->present = true;
    return pdPASS;
}

inline BaseType_t xQueuePeek(QueueHandle_t queue,
    void *item,
    TickType_t)
{
    if(!queue || !item || !queue->present){return pdFALSE;}
    memcpy(item, queue->storage, queue->item_size);
    return pdPASS;
}

inline BaseType_t xQueueOverwriteFromISR(QueueHandle_t queue,
    const void *item,
    BaseType_t *)
{
    return xQueueOverwrite(queue, item);
}

inline BaseType_t xQueuePeekFromISR(QueueHandle_t queue, void *item)
{
    return xQueuePeek(queue, item, 0U);
}

inline BaseType_t xQueueSendToBack(QueueHandle_t queue,
    const void *item,
    TickType_t)
{
    return xQueueOverwrite(queue, item);
}

inline BaseType_t xQueueReceive(QueueHandle_t queue,
    void *item,
    TickType_t)
{
    if(xQueuePeek(queue, item, 0U) != pdPASS){return pdFALSE;}
    queue->present = false;
    return pdPASS;
}

inline BaseType_t xQueueSendToBackFromISR(QueueHandle_t queue,
    const void *item,
    BaseType_t *)
{
    return xQueueOverwrite(queue, item);
}

inline BaseType_t xQueueReceiveFromISR(QueueHandle_t queue,
    void *item,
    BaseType_t *)
{
    return xQueueReceive(queue, item, 0U);
}

inline UBaseType_t uxQueueMessagesWaiting(QueueHandle_t queue)
{
    return queue && queue->present ? 1U : 0U;
}

#endif
