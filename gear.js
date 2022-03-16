// Copyright: Qianyan Cai
// License: GPL v3

// 渐开线齿轮 Involute Gear
function GearI({
	M, // 模数
	A = 20, // 压力角度数
	Z, // 齿数
	S = 0, // 变位系数
	I = false, // 内齿轮
	tickn = 240, // 齿廓总步进数
	zoom, // 像素比例
	size, // 预估像素
}) {
	if ((M = round(M * 8) >> 3) <= 0) throw 'err M'
	if (!((A = roundepsi(A)) > 0)) throw 'err A'
	if (Z != (Z |= 0) || Z < 6) throw 'err Z'
	if (!((S = roundepsi(S)) >= 0)) throw 'err S'
	tickn = max(ceil(tickn / Z / 2), 3) * Z * 2 // 齿廓步进数，齿数整倍数

	let TA = (A * PI) / 180 // 压力角
	let TP = PI2 / Z // 齿距角
	let TQ = PI / Z // 齿顶根角
	let TB = (PI + 4 * S * tan(TA)) / Z + (tan(TA) - TA) * 2 // 基角（依据基点，齿厚角+两侧分度基角）

	let P = M * Z * 0.5 // 分度圆半径
	let B = P * cos(TA) // 基圆半径
	let U = P + M * (S + (I ? 1.25 : 1)) // 齿顶半径
	let F = P + M * (S - (I ? 1 : 1.25)) // 齿根半径
	if (I && F < B) F = B
	// let Q = P + M * S // 中分圆半径
	// U = Q + M * (I ? 1.25 : 1) // 齿顶半径
	// F = Q - M * (I ? 1 : 1.25) // 齿根半径
	// let CQ = sqrt(Q * Q - B * B) / B // 中分渐开角
	// TB = TQ + (CQ - acos(B / Q)) * 2 // 基角（依据基点，中分齿厚角+两侧中分基角）

	let CU = sqrt(U * U - B * B) / B // 齿顶渐开角
	let CF = sqrt(max(F * F - B * B, 0)) / B // 齿根渐开角
	let CC = ((CU - CF) * Z * 2) / tickn // 步进渐开角
	let CX = (T, C) => B * cos(T + C) + B * C * sin(T + C) // 齿廓点X
	let CY = (T, C) => B * sin(T + C) - B * C * cos(T + C) // 齿廓点Y
	let TZ = z => (z * PI2) / Z - TB / 2 // 齿起始角（依据基点）

	let T2 = (T, gear2) => (T / gear2.Z) * Z * (I == gear2.I ? -1 : 1)
	let E = gear2 => abs(P + gear2.P * (I == gear2.I ? -1 : 1))

	zoom ??= ceil(+size || min(size.width, size.height)) / (B * 2 + M * 3)
	size = ceil(U * zoom * 2)
	Object.assign(this, { zoom, size, M, A, Z, S, I, B, P, U, F })
	Object.assign(this, { TP, TQ, TZ, T2, E })

	// 参数显示
	function params(T) {
		return _`M${M} A${A}__Z${Z} S${S}__B${B}{1} P${P}{1}__F${F}{1} U${U}{1}__`
	}
	console.log(...params().split('__'), _`${TB}{5} tn${tickn}`, F < B ? 'F<B' : '')

	this.$ = ({ canvas, midx, midy, param }) => {
		let $ = canvas.getContext('2d')
		let x = midx ?? canvas.width / 2 // 齿心X
		let y = midy ?? canvas.height / 2 // 齿心Y
		let $param = T => (param.textContent = params(T).replace(/__/g, '\n'))

		function $$({ color = '#000', opa = '', thick = 1, dash } = {}, fill) {
			$.beginPath(), ($.lineWidth = thick)
			opa.length == 1 && /#....../.test(color) && (opa += opa)
			opa.length > 0 && /#....(....)?$/.test(color) && (color = color.slice(0, -opa.length))
			fill ? ($.fillStyle = color + opa) : ($.strokeStyle = color + opa)
			dash && $.setLineDash(dash)
		}
		function $$$(fill) {
			fill ? $.fill() : $.stroke(), ($.lineWidth = 1)
			fill ? ($.fillStyle = '#000') : ($.strokeStyle = '#000')
			$.setLineDash([])
		}
		// 画齿心
		function $O(style) {
			$$({ color: '#ccc', ...style })
			$.arc(x, y, 1, 0, PI2), $$$()
		}
		// 画基圆
		function $B(style) {
			$$({ color: '#999', dash: [1, 3], ...style })
			$.arc(x, y, B * zoom, 0, PI2), $$$()
		}
		// 画中分圆
		function $P(style) {
			$$({ color: '#ccc', ...style })
			$.arc(x, y, P * zoom, 0, PI2), $$$()
		}
		// 画齿
		function $C(T, z = 0, style, f = true) {
			$$({ color: '#000', ...style })
			let to
			T += TZ(z)
			for (let C = CF; C <= CU + EPSI; C += CC)
				(to = to ? $.lineTo : $.moveTo).call($, x + CX(T, C) * zoom, y + CY(T, C) * zoom)
			for (let C = CU; C >= CF - EPSI; C -= CC)
				$.lineTo(x + CX(T + TB, -C) * zoom, y + CY(T + TB, -C) * zoom)
			if (f) {
				if (F < B)
					$.lineTo(x + F * cos(T + TB) * zoom, y + F * sin(T + TB) * zoom),
						$.lineTo(x + F * cos(T + TP) * zoom, y + F * sin(T + TP) * zoom)
				$.lineTo(x + CX(T + TP, CF) * zoom, y + CY(T + TP, CF) * zoom)
			}
			$$$()
		}
		// 画全部齿
		function $CZ(T, style) {
			for (let z = 0; z < Z; z++) $C(T, z, style)
		}
		return Object.assign({ param: $param, x, y, O: $O, B: $B, P: $P, C: $C, CZ: $CZ })
	}
}
