import pwColors from './../../assets/pwColors.json'

function getEventsByCategory(events, category) {
    if (!category) {
        return events
    }

    if ('displayAll' in category && category.displayAll === true) {
        return events
    }

    if (!('name' in category)) {
        return events
    }

    return getEventsByCategoryName(events, category.name.toLowerCase())
}

function getEventsByCategoryName(events, categoryName) {
    if (categoryName === 'all') {
        return events
    }

    return events.filter((e) => e.category === categoryName)
}

const levels = ['info', 'warning', 'error', 'stale']
const severityColors = [
    pwColors.miętowy,
    pwColors.słoneczny,
    pwColors.morelowy,
    pwColors.śliwkowy,
]

function getEventSeverity(event) {
    return levels.indexOf(event.level)
}

function getEventSeverityColor(severity) {
    if (severity < 0 || severity > severityColors.length) {
        return severity[severityColors.length - 1]
    }

    return severityColors[severity]
}

function getHighestEventSeverity(events) {
    return getEventSeverity(
        events.reduce((prev, curr) =>
            getEventSeverity(prev) > getEventSeverity(curr) ? prev : curr
        )
    )
}

function getSeverityColorForCategory(events, category) {
    return getEventSeverityColor(
        getHighestEventSeverity(getEventsByCategoryName(events, category))
    )
}

export {
    getEventsByCategory,
    getEventsByCategoryName,
    getHighestEventSeverity,
    getEventSeverityColor,
    getSeverityColorForCategory,
}
