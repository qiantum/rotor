let { min, max, abs, floor, ceil, round, PI, cos, sin, sqrt } = Math
let PI2 = (Math.PI2 = PI + PI)
let EPSI = (Number.EPSILON = 1 / (1 << 12))
let atan, diff, diffAbs, cross, area

function Rotor({
	N, // 转角数
	e, // 偏心距
	P = (N - 1) * 0.8, // 转子角半径 / 偏心距
	Q = P, // 转子腰半径 / 偏心距
	BP = 0.1, // 缸体转子间隙 / 偏心距
	Tick: Tickn = 16, // 每角旋转步进
	Dfast, // 快速计算转子型线
	size, // 预估像素
}) {
	let PIN = PI / N
	let N2 = N + N
	Tickn = floor(min(Tickn * N2, 160) / N2) * N2 // 每角旋转步进
	size = ceil(+size || min(size.clientWidth, size.clientHeight))
	e ??= floor(size / (2.3 * (P + N + 3))) // 偏心距
	let R = e * N // 转子大节圆半径
	let r = R - e // 曲轴小节圆半径
	P = e * (P + N + 2) // 转子角半径
	Q = e * (Q + N) // 转子腰半径
	BP *= e // 缸体转子间隙

	let Tn = n => (n * PI2) / N
	let Tst = st => (st * PI) / (N - 1)
	let tT = (T, int = round) => int((Tickn * T) / PI2) % Tickn // [0,Tickn)
	let Tick_ = (this.Tick_ = [...Array.seq(0, Tickn)].map(t => (t / Tickn) * PI2))
	let Tick = (this.Tick = Tick_.slice(0, Tickn))

	// 缸体型线
	let BB = Tick_.map(T => [
		e * cos(T * N - PI) + (P + BP) * cos(T),
		e * sin(T * N - PI) + (P + BP) * sin(T),
	])
	size = BB.reduce((v, [X, Y]) => max(v, abs(X), abs(Y)), 0)
	// 缸体腰线
	let BQt = [...Array.seq(Tickn - Tickn / N2, Tickn - 1), ...Array.seq(0, Tickn / N2)]
	// 缸体顶线
	let TBP = Tst(1)
	let BPt = [...Array.seq(Tick.binFind(TBP - PIN), Tick.binFind(TBP + PIN))]

	// 转子型线、即缸体绕转子心的内包络线
	let D, DD
	if (Dfast) {
		DD = new Array(Tickn).fill(P)
		for (let T of Tick)
			for (let B of BQt.map(t => Tick[t])) {
				let X = P * cos(B + T) - e * cos(B * N + T) - e * cos(T * (N - 1))
				let Y = P * sin(B + T) - e * sin(B * N + T) + e * sin(T * (N - 1))
				let t = tT(atan(X, Y), ceil)
				DD[t] = min(DD[t], sqrt(X * X + Y * Y))
			}
		DD.push(Q)
		for (let t = 0; t < Tickn; t++)
			if (t % (Tickn / N2) == 0) DD[t] = t % (Tickn / N) == 0 ? Q : P
			else DD[t] = min(DD[t], DD[t + 1]) * 0.375 + max(DD[t], DD[t + 1]) * 0.625

		D = function* (T, dt = Array.seq(0, Tickn)) {
			let x = e * cos(T * N)
			let y = e * sin(T * N)
			for (let t of dt) yield [x + DD[t] * cos(T + Tick_[t]), y + DD[t] * sin(T + Tick_[t])]
		}
	} else {
		for (let B of Tick) {
			// 缸体转动
			let bb = Tick.map(T => {
				let X = P * cos(B + T) - e * cos(B * N + T) - e * cos(T * (N - 1))
				let Y = P * sin(B + T) - e * sin(B * N + T) + e * sin(T * (N - 1))
				let A = atan(X, Y) // [0,PI2) C==0 沿T严格递增 [0,PI2] C>0 沿T循环严格递增
				return [A, sqrt(X * X + Y * Y), X, Y, DD?.binFind(A, 0, -1)]
			})
			if (DD != (DD ??= [...bb])) continue
			let S = []
			// 对缸体每段
			for (let bt = 0; bt < Tickn; bt++) {
				let [a_, d_, x_, y_, t_] = bb[bt]
				let [a, d, x, y, t] = bb[bt + 1] || bb[0]
				t_--, (t %= DD.length) // 对应转子连续段
				for (let T_ = t_, T1, T; T_ != t; T_ = T) {
					let [A_, D_, X_, Y_] = DD[T_]
					let [A, D, X, Y] = DD[(T = T1 = T_ + 1)] || DD[(T = 0)]
					// 求交点和半径
					let [SX, SY, abc, abd, cda, cdb] = cross(X_, Y_, X, Y, x_, y_, x, y)
					if (T != t && cdb < -EPSI && T) S.push([1 / 0, 0, 0, 0, T]) // 去掉长半径转子点
					if (T_ == t_ && abc > EPSI) S.push([a_, d_, x_, y_, T1]) // 短半径缸体点
					if (T == t && abd > EPSI) S.push([a, d, x, y, T1]) // 短半径缸体点
					if (SX != null) S.push([atan(SX, SY), sqrt(SX * SX + SY * SY), SX, SY, T1]) //交点
				}
			}
			S.sort((s1, s2) => s1[4] - s2[4] || s1[0] - s2[0]) // 保持角度有序
			let t = 0 // 加减点
			for (let s of S)
				if (s[0] == 1 / 0) DD.splice(s[4] + t--, 1)
				else DD.splice(s[4] + t++, 0, s), s.length--
			DD[DD.push([...DD[0]]) - 1][0] += PI2 // 闭合
			t = 0 // 合并重合点
			for (let D of DD)
				if (D[0] < DD[t][0]) throw 'err a'
				else if (D[0] - DD[t][0] > EPSI && abs(D[1] - DD[t][1]) > EPSI) DD[++t] = D
			DD.length = ++t
		}

		D = function* (T, dt = Array.seq(0, Tickn)) {
			let s = dt.values?.() ?? dt
			let tt = s.next().value
			if (tt == null) return
			let x = e * cos(T * N)
			let y = e * sin(T * N)
			let d = DD.binFind(Tick_[tt], 0)
			for (let t of s)
				do
					for (let A, tT = Tick_[t] + (tt > t && PI2); (A = DD[d]?.[0]) <= tT; d++)
						yield [x + DD[d][1] * cos(T + A), y + DD[d][1] * sin(T + A)]
				while (tt > (tt = t) && !(d = 0))
		}
	}
	console.log(`D in ${DD.length - 1} details`)

	// 工作容积，总容积，总体积
	let V, K, VB, VV, KB, KK
	{
		let v = area(BQt.map(t => BB[t])) - area(D(0, BQt))
		V = area(BPt.map(t => BB[t])) - area(D(TBP, BQt)) - v
		VB = area(BB)
		VV = VB - area(D(0))
		;(K = V / v), (KB = VB / V), (KK = VV / V)
		console.log(`Vmin ${v | 0}  Vmax ${V | 0}  K ${K.toFixed(1)}`)
	}
	console.log(`Ktotal ${KK.toFixed(1)} Kblock ${KB.toFixed(1)}`)

	Object.assign(this, { N, e, R, r, P, Q, BP, D, V, V, K, KK, KB, Tn, Tst, size })

	this.$ = ({ canvas, centerx, centery }) => {
		let $ = canvas.getContext('2d')
		let $x = centerx ?? canvas.clientWidth / 2
		let $y = centery ?? canvas.clientHeight / 2
		let $X = T => $x + e * cos(T * N) // 转子心X
		let $Y = T => $y + e * sin(T * N) // 转子心Y

		function $$({ color = '#000', opa = '', thick = 1 } = {}) {
			$.beginPath(), ($.strokeStyle = color + opa), ($.lineWidth = thick)
		}
		function $$$() {
			$.stroke(), ($.strokeStyle = '#000'), ($.lineWidth = 1)
		}
		// 画曲轴小圆
		function $r(style) {
			$$(style), $.arc($x, $y, r, 0, PI2), $$$()
		}
		// 画偏心线
		function $Rr(T, style) {
			$$({ color: '#ccc', thick: 4, ...style })
			$.moveTo($x, $y), $.lineTo($X(T), $Y(T)), $$$()
		}
		// 画转子大圆
		function $R(T, style) {
			$$({ color: '#999', ...style }), $.arc($X(T), $Y(T), R, 0, PI2), $$$()
		}
		// 画转子大圆凸包
		function $RR(style) {
			$$({ color: '#999', opa: '7', ...style }), $.arc($x, $y, R + e, 0, PI2), $$$()
		}
		// 画转子角
		function $P(T, n = 0, O, style) {
			T += Tn(n)
			$$({ color: '#00f', ...style })
			let X = $X(T) + P * cos(T + PIN)
			let Y = $Y(T) + P * sin(T + PIN)
			BP && $.arc(X, Y, BP, 0, PI2)
			$.moveTo(X, Y), $.lineTo($X(T) + O * cos(T + PIN), $Y(T) + O * sin(T + PIN))
			$$$()
		}
		// 画转子腰
		function $Q(T, n = 0, O, style) {
			T += Tn(n)
			$$({ color: '#0f0', ...style })
			$.moveTo($X(T) + Q * cos(T), $Y(T) + Q * sin(T))
			$.lineTo($X(T) + O * cos(T), $Y(T) + O * sin(T))
			$$$()
		}
		// 画转子全部角
		function $PN(T, O = R, style) {
			for (let n = 0; n < N; n++) $P(T, n, O?.[n] ?? O?.[O?.length - 1] ?? O, style)
		}
		// 画转子全部腰
		function $QN(T, O = [0, R], style) {
			for (let n = 0; n < N; n++) $Q(T, n, O?.[n] ?? O?.[O?.length - 1] ?? O, style)
		}
		// 画转子腰凸包
		function $QQ(style) {
			$$({ color: '#9f9', ...style })
			for (let T of Tick_) {
				let to = T ? $.lineTo : $.moveTo
				to.call($, $X(T) + Q * cos(T), $Y(T) + Q * sin(T))
			}
			$$$()
		}
		// 画缸体
		function $BB(style) {
			$$({ color: '#00f', ...style })
			let to
			for (let [X, Y] of BB) (to = to ? $.lineTo : $.moveTo).call($, $x + X, $y + Y)
			$$$()
		}
		// 画转子
		function $DD(T, style) {
			$$(style)
			let to
			for (let [X, Y] of D(T)) (to = to ? $.lineTo : $.moveTo).call($, $x + X, $y + Y)
			$$$()
		}
		return {
			x: $x,
			y: $y,
			r: $r,
			Rr: $Rr,
			R: $R,
			RR: $RR,
			P: $P,
			Q: $Q,
			PN: $PN,
			QN: $QN,
			QQ: $QQ,
			BB: $BB,
			DD: $DD,
		}
	}
}

atan = function (x, y) {
	if (abs(x) < EPSI) return y > 0 ? PI * 0.5 : PI * 1.5
	return Math.atan(y / x) + (x < 0 ? PI : y < 0 ? PI2 : 0)
}
diff = v => ((v %= PI2) > PI ? v - PI2 : v < -PI ? v + PI2 : v)
diffAbs = v => ((v = abs(v) % PI2) > PI ? PI2 - v : v)

Array.seq = function* (from, to) {
	for (let i = from; i <= to; i++) yield i
}
Array.prototype.binFind = function (v, prop, epsi = EPSI, neg) {
	let l = 0
	for (let h = this.length - 1, m, c; l <= h; ) {
		;(m = (l + h) >>> 1), (c = v - (prop != null ? this[m][prop] : this[m]))
		if (c >= -epsi && c <= epsi) return m
		c < 0 ? (h = m - 1) : (l = m + 1)
	}
	return neg ? ~l : l
}
cross = function (ax, ay, bx, by, cx, cy, dx, dy, co) {
	let abc = (ax - cx) * (by - cy) - (ay - cy) * (bx - cx)
	let abd = (ax - dx) * (by - dy) - (ay - dy) * (bx - dx)
	let cda = (cx - ax) * (dy - ay) - (cy - ay) * (dx - ax)
	let cdb = abc - abd + cda
	if (abc * abd >= 0 || cda * cdb >= 0) {
		if (!co || (abc && abd)) return [null, null, abc, abd, cda, cdb]
		return cda == 0 ? [ax, ay, abc, abd, cda, cdb] : [bx, by, abc, abd, cda, cdb]
	}
	let t = cda / (abd - abc)
	return [ax + t * (bx - ax), ay + t * (by - ay), abc, abd, cda, cdb]
}
area = function (s) {
	let [x0, y0] = (s = s.values?.() ?? s).next().value
	let [xx, yy] = [x0, y0]
	let a = 0
	for (let [x, y] of s) (a += (xx - x) * (yy + y)), (xx = x), (yy = y)
	return (a + (xx - x0) * (yy + y0)) / 2
}
