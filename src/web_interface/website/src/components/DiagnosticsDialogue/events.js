import pwColors from './../../assets/pwColors.json'

// region Constants

const levels = ['info', 'warning', 'error', 'stale', 'unknown']
const levelsColors = {
    info: pwColors.miętowy,
    warning: pwColors.słoneczny,
    error: pwColors.morelowy,
    stale: pwColors.śliwkowy,
    unknown: pwColors.grafitowy,
}

// endregion

// region Filtering & Grouping

function groupEventsByCategory(events, categories) {
    let groups = {}

    for (const category of categories) {
        groups[category.id] = []
    }

    for (const event of events) {
        const category = 'category' in event ? event.category : 'other'

        if (category in groups) {
            groups[category].push(event)
        } else {
            groups['other'].push(event)
        }

        if (category !== 'all') {
            groups['all'].push(event)
        }
    }

    return groups
}

function groupEventsByLevel(events) {
    let groups = {}

    for (const level of levels) {
        groups[level] = []
    }

    for (const event of events) {
        const level = 'level' in event ? event.level : 'unknown'

        if (level in levels) {
            groups[level].push(event)
        } else {
            groups['unknown'].push(event)
        }
    }

    return groups
}

// endregion

// region Categories

function getCategoryByIndex(categories, index) {
    if (index > categories.length || index < 0) {
        return { id: 'all' }
    }

    return categories[index]
}

// endregion

// region Levels

function getEventLevel(event) {
    const level = 'level' in event ? event.level : 'unknown'

    if (levels.includes(level)) {
        return level
    }

    return 'unknown'
}

function getEventLevelColor(event) {
    return levelsColors[getEventLevel(event)]
}

function getHighestLevelEvent(events) {
    if (events.length === 0) {
        return { level: 'info' }
    }

    if (events.length === 1) {
        return events[0]
    }

    return events.reduce((previous, current) =>
        levels.indexOf(getEventLevel(previous)) >
        levels.indexOf(getEventLevel(current))
            ? previous
            : current
    )
}

function getHighestLevelEventColor(events) {
    return getEventLevelColor(getHighestLevelEvent(events))
}

// endregion

export {
    groupEventsByCategory,
    groupEventsByLevel,
    getCategoryByIndex,
    getEventLevel,
    getEventLevelColor,
    getHighestLevelEvent,
    getHighestLevelEventColor,
}
