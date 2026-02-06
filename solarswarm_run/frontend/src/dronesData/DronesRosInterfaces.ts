export type Field = {
    name: string
    type: string
    isArray: boolean
}

export type ParsedInterface = {
    request: { inputs: Field[] }
    response: { outputs: Field[] }
}

export type NamedParsedInterface = {
    serviceName: string
    serviceType: string
    parsed: ParsedInterface
}

export function parseInterface(definition: string): ParsedInterface {
    const lines = definition
        .split('\n')
        .map(l => l.trim())
        .filter(l => l && !l.startsWith('#'))

    const sepIndex = lines.indexOf('---')

    const requestLines =
        sepIndex === -1 ? lines : lines.slice(0, sepIndex)
    const responseLines =
        sepIndex === -1 ? [] : lines.slice(sepIndex + 1)

    function parseLines(lines: string[]): Field[] {
        return lines.map(line => {
            const [rawType, name] = line.split(/\s+/)

            const isArray = rawType.endsWith('[]')
            const type = isArray ? rawType.slice(0, -2) : rawType

            return { name, type, isArray }
        })
    }

    return {
        request: { inputs: parseLines(requestLines) },
        response: { outputs: parseLines(responseLines) }
    }
}

export function prettyPrintInterface(svc: NamedParsedInterface): string {
    const lines: string[] = []

    const formatField = (f: { name: string; type: string; isArray: boolean }) =>
        `  ${f.name} : ${f.type}${f.isArray ? '[]' : ''}`

    lines.push(`SERVICE  ${svc.serviceName}`)
    lines.push(`TYPE     ${svc.serviceType}`)
    lines.push('')

    lines.push('REQUEST')
    svc.parsed.request.inputs.length
        ? svc.parsed.request.inputs.forEach(f => lines.push(formatField(f)))
        : lines.push('  (no inputs)')

    lines.push('')
    lines.push('RESPONSE')

    svc.parsed.response.outputs.length
        ? svc.parsed.response.outputs.forEach(f => lines.push(formatField(f)))
        : lines.push('  (no outputs)')

    return lines.join('\n')
}


