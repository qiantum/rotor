let { min, max, floor, ceil, abs, sqrt, PI, cos, sin } = Math
let PI2 = (Math.PI2 = PI + PI)
let EPSI = (Number.EPSILON = 1 / (1 << 12))
let atan, diff, diffAbs, cross, area

function Rotor({ N = 2, e, P = (N - 1) * 0.8, Q = P, PC = 0.1, Tick: Tickn = 16, size }) {
	Tickn = floor(min(Tickn * N * 2, 100) / N / 2) * N * 2
	let Tick_ = (this.Tick_ = [])
	for (let i = 0; i <= Tickn; i++) Tick_.push((i / Tickn) * PI2)
	let Tick = (this.Tick = [...Tick_])
	Tick.length--
	size = ceil(+size || min(size.clientWidth, size.clientHeight))

	let PIN = PI / N
	e ??= size / (2.3 * (P + N + 3)) // 偏心距
	let R = e * N // 转子大节圆半径
	let r = R - e // 曲轴小节圆半径
	P = e * (P + N + 2) // 顶半径
	Q = e * (Q + N) // 腰半径
	PC *= e // 缸体转子间隙
	let Tn = n => (n * PI2) / N
	let Tst = st => (st * PI) / (N - 1)

	// 缸体型线
	let CC = Tick_.map(T => [
		e * cos(T * N - PI) + (P + PC) * cos(T),
		e * sin(T * N - PI) + (P + PC) * sin(T),
	])

	// 转子型线、即缸体绕转子心的内包络线
	let DD = null
	for (let C of Tick) {
		// 缸体转动
		let cc = Tick.map(T => {
			let X = P * cos(C + T) - e * cos(C * N + T) - e * cos(T * (N - 1))
			let Y = P * sin(C + T) - e * sin(C * N + T) + e * sin(T * (N - 1))
			let A = atan(X, Y) // [0,PI2) C==0 沿T严格递增 [0,PI2] C>0 沿T循环严格递增
			return [A, sqrt(X * X + Y * Y), X, Y, DD?.binFind(A, 0, true, -1)]
		})
		if (DD != (DD ??= [...cc])) continue
		let S = []
		for (let ct = 0; ct < Tickn; ct++) {
			// 对缸体每条线段
			let [a_, d_, x_, y_, t_] = cc[ct]
			let [a, d, x, y, t] = cc[ct + 1] || cc[0]
			t_--, (t %= DD.length) // 缸体每条线段
			for (let T_ = t_, T1, T; T_ != t; T_ = T) {
				let [A_, D_, X_, Y_] = DD[T_]
				let [A, D, X, Y] = DD[(T = (T1 = T_ + 1) % DD.length)]
				// 缸体线段角对应的转子连续线段，求交点和半径
				let [SX, SY, abc, abd, cda, cdb] = cross(X_, Y_, X, Y, x_, y_, x, y)
				if (T != t && cdb < -EPSI && T) S.push([1 / 0, 0, 0, 0, T]) //减半径长转子点
				if (T_ == t_ && abc > EPSI) S.push([a_, d_, x_, y_, T1]) //加半径短缸体点
				if (T == t && abd > EPSI) S.push([a, d, x, y, T1]) //加半径短缸体点
				if (SX != null) S.push([atan(SX, SY), sqrt(SX * SX + SY * SY), SX, SY, T1]) //加交点
			}
		}
		S.sort((a, b) => a[4] - b[4] || a[0] - b[0]) // 保持角度有序
		let t = 0 // 加减点
		for (let s of S)
			if (s[0] == 1 / 0) DD.splice(s[4] + t--, 1)
			else DD.splice(s[4] + t++, 0, s), s.length--
		t = 0 // 合并重合点
		for (let D of DD)
			if (D[0] < DD[t][0]) throw 'err a'
			else if (D[0] - DD[t][0] > EPSI && abs(D[1] - DD[t][1]) > EPSI) DD[++t] = D
		DD.length = ++t
	}
	DD[DD.push([...DD[0]]) - 1][0] += PI2
	console.log(`DD in ${DD.length} details`)

	// 工作容积，总容积，总体积
	let V, K, VC, VV, KC, KK
	{
		let c = CC.slice(-Tickn / N / 2 - 1).concat(CC.slice(0, Tickn / N / 2 + 1))
		let d = DD.slice(DD.binFind(PI2 - PIN, 0, true))
		d = d.concat(DD.slice(0, DD.binFind(PIN, 0, true) + 1))
		let v = area(c) - area(d.map(([A, D, X, Y]) => [X, Y]))
		let t = Tst(1)
		c = CC.slice(Tick.binFind(t - PIN, null, true), Tick.binFind(t + PIN, null, true) + 1)
		d = d.map(([A, D]) => [e * cos(t * N) + D * cos(t + A), e * sin(t * N) + D * sin(t + A)])
		;(V = area(c) - area(d) - v), (K = V / v)
		;(VC = area(CC)), (KC = VC / V)
		;(VV = VC - area(DD.map(([A, D, X, Y]) => [X, Y]))), (KK = VV / V)
	}
	console.log(`Vmin ${V | 0}  Vmax ${V | 0}  K ${K.toFixed(1)}`)
	console.log(`Ktotal ${KC.toFixed(1)} Kwhole ${KK.toFixed(1)}`)

	Object.assign(this, { N, e, R, r, P, Q, V, V, K, KK, KC, Tn, Tst })

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
		// 画转子顶
		function $P(T, n = 0, O, style) {
			T += (n * PI2) / N
			$$({ color: '#00f', ...style })
			let X = $X(T) + P * cos(T + PIN),
				Y = $Y(T) + P * sin(T + PIN)
			PC && $.arc(X, Y, PC, 0, PI2)
			$.moveTo(X, Y)
			$.lineTo($X(T) + O * cos(T + PIN), $Y(T) + O * sin(T + PIN))
			$$$()
		}
		// 画转子腰
		function $Q(T, n = 0, O, style) {
			T += (n * PI2) / N
			$$({ color: '#0f0', ...style })
			$.moveTo($X(T) + Q * cos(T), $Y(T) + Q * sin(T))
			$.lineTo($X(T) + O * cos(T), $Y(T) + O * sin(T))
			$$$()
		}
		// 画转子全部顶
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
		function $CC(style) {
			$$({ color: '#00f', ...style })
			let to
			for (let [X, Y] of CC) (to = to ? $.lineTo : $.moveTo).call($, $x + X, $y + Y)
			$$$()
		}
		// 画转子
		function $DD(T, style) {
			$$(style)
			for (let [A, D] of DD)
				(A ? $.lineTo : $.moveTo).call($, $X(T) + D * cos(T + A), $Y(T) + D * sin(T + A))
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
			CC: $CC,
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

Array.prototype.binFind = function (v, prop, at, epsi = EPSI) {
	let l = 0
	for (let h = this.length - 1, m, c; l <= h; ) {
		;(m = (l + h) >>> 1), (c = v - (prop != null ? this[m][prop] : this[m]))
		if (c >= -epsi && c <= epsi) return m
		c < 0 ? (h = m - 1) : (l = m + 1)
	}
	return at ? l : ~l
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
	let a = 0,
		[xx, yy] = s[s.length - 1]
	for (let [x, y] of s) (a += (xx - x) * (y + yy)), (xx = x), (yy = y)
	return a / 2
}
