// https://stackoverflow.com/a/68764864

// change: indicator . and f are optional
let FORMATTER = function (obj, fmt) {
	/* implements things using (Number).toFixed:
       ${1/3}{.2f} -> 0.33
       ${1/3}{.0f} -> 1
       ${1/3}{%} -> 33%
       ${1/3}{.3%} -> 33.333%
       ${1/3}{d} -> 0
       ${{a:1/3,b:1/3}}{.2f} -> {"a":0.33, "b":0.33}
       ${{a:1/3,b:1/3}}{*:'.2f',b:'%'} -> {"a":0.33, "b":'33%'}  //TODO not implemented
       ${[1/3,1/3]}{.2f} -> [0.33, 0.33]
       ${someObj} -> if the object/class defines a method [Symbol.FTemplate](){...}, 
                     it will be evaluated; alternatively if a method [Symbol.FTemplateKey](key){...} 
                     that can be evaluated to a fmt string; alternatively in the future 
                     once decorators exist, metadata may be appended to object properties to derive 
                     formats //TODO not implemented
    */
	let _, fracDigits, percent
	if (fmt === undefined) {
		if (typeof obj === 'string') return obj
		else return JSON.stringify(obj)
	} else if (obj instanceof Array) return '[' + obj.map(x => FORMATTER(x, fmt)) + ']'
	else if (typeof obj === 'object' && obj !== null /*&&!Array.isArray(obj)*/)
		return JSON.stringify(
			Object.fromEntries(Object.entries(obj).map(([k, v]) => [k, FORMATTER(v, fmt)]))
		)
	else if ((matches = fmt.match(/^(\d+)?(%)?$/))) [_, fracDigits = '0', percent] = matches
	else throw 'error format ' + fmt

	if (obj === null) return 'null'
	if (obj === undefined) {
		// one might extend the above syntax to
		// allow for example for .3f? -> "undefined"|"0.123"
		return 'undefined'
	}

	if (percent) obj *= 100

	fracDigits = parseFloat(fracDigits)
	return obj.toFixed(fracDigits) + (percent ? '%' : '')
}

let _ = function (strs, ...args) {
	/* usage: F`Demo: 1+1.5 = ${1+1.5}{.2f}` 
          --> "Demo: 1+1.5 = 2.50" 
    */
	let R = strs[0]
	args.forEach((arg, i) => {
		let fmt = strs[i + 1].match(/^\{(.*)\}/)
		R += FORMATTER(arg, fmt?.[1]) + strs[i + 1].slice(fmt?.[0].length)
	})
	return R
}
globalThis._ = _
console._ = (strs, ...args) => console.log(_(strs, ...args))
