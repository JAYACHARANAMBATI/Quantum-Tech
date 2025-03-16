import { Link } from "react-router-dom";

const Communication = () => (
  <div className="page-container">
    <section className="hero-section">
      <h1 className="hero-title">Communication Systems</h1>
      <p className="hero-subtitle">
        Explore the evolution and future of communication technologies that connect our world,
        from traditional networks to cutting-edge quantum communication.
      </p>
    </section>

    <div className="feature-grid">
      <div className="feature-card">
        <i className="fas fa-broadcast-tower fa-2x mb-3" style={{ color: "var(--primary)" }}></i>
        <h2>Wireless Technologies</h2>
        <p>Understand how 5G, Wi-Fi 6, and other wireless technologies are transforming connectivity.</p>
      </div>

      <div className="feature-card">
        <i className="fas fa-satellite fa-2x mb-3" style={{ color: "var(--primary)" }}></i>
        <h2>Satellite Communications</h2>
        <p>Learn about satellite networks providing global coverage for communications, navigation, and more.</p>
      </div>

      <div className="feature-card">
        <i className="fas fa-atom fa-2x mb-3" style={{ color: "var(--primary)" }}></i>
        <h2>Quantum Communication</h2>
        <p>Discover how quantum entanglement enables secure, unhackable communication channels.</p>
      </div>

      <div className="feature-card">
        <i className="fas fa-shield-alt fa-2x mb-3" style={{ color: "var(--primary)" }}></i>
        <h2>Security Protocols</h2>
        <p>Explore the protocols and technologies that protect data transmission across networks.</p>
      </div>
    </div>

    <section className="detail-section">
      <h2>Evolution of Communication Systems</h2>
      <p>
        Communication systems have evolved dramatically over the centuries, from primitive signal fires 
        and drum beats to today's sophisticated digital networks that connect billions of people worldwide.
        Each generation of technology has brought greater speed, capacity, and functionality.
      </p>

      <div className="row align-items-center my-4">
        <div className="col-md-6">
          <img 
            src="data:image/jpeg;base64,/9j/4AAQSkZJRgABAQAAAQABAAD/2wCEAAkGBxIREBUQEhIWFRUWFRIVFRUVFRcYFRYVFhUXGBUWFhUYHyggGB4lGxUYITIhJSorLi4uGB8zODMsNygtLysBCgoKDg0OGxAQGi0lHyUtLS0tLy4vLi81LS8tLS0tLS0tKy0vLSsvLS0uNS0tLS0tNS0tLS0tLS4tLS0tLzUtLf/AABEIAIYBeQMBIgACEQEDEQH/xAAbAAEAAgMBAQAAAAAAAAAAAAAABQYBAwQCB//EAEsQAAIBAwIDBAYFBwkGBwEAAAECAwAEERIhBRMxBiJBURQyM2FxgSNCUmKRBxVykqGxskNTVHOClLPR0xYkY3WTlTREVYOiwdQl/8QAGAEBAQEBAQAAAAAAAAAAAAAAAAECBAP/xAAlEQEAAgEEAQQCAwAAAAAAAAAAARECAxIhMYEEE1GRQXEU4fD/2gAMAwEAAhEDEQA/APuNKUoFKUoFKUoFKUoFKxmuGfjECHSZAW+wmZJP1EBb9lB30qM/Obt6ltKfIvojHzDNqH6tFa7bqkCeXfeT8e6lBJ0qMFvdnrPEP0YGz+LSn91PQrk/+ax+jCg/eTQSdKjPQrnwuh84VP7iKGC7HSaE/GBwfxEv/wBUEnSoxpLtT7OFx7pHjP4FGH7RWfzqy+0tplHmFWQfIRMzH8KCSrDVx23FoJDpWRdX2CdL/NGww/CuqSgzmsM4G5OB5npVc48k4uVeEN7ApqCagC1zADt0zo1n4AmozjD3bRyQtziNM6qVgDGUiXCiQquEHL3BGnUTt00mQi8ZoTVaveIXOt40DjDzYcwO6BOQWiIKr3/pPBd9seWYlLi91u4a6y0UCorxBo9QuGWVwBCmk8tge8FJHgQNqs8L3msBgc4PQ4PuPl+2qnxGW4heXlrKxY26iVYwWOIn1MxWJx1VRlYzgsBhRuNPCoJZbO5NwkkTzSRO4WJi2TbWof6LGXXUrKy75AYVEXOs1Smu71I4FjiES6ZfUjk0FlkAizEIpHijZMto7pXONQIrqhuL0RuWZtTQhxmE/RycwjQoRCd1x1ViD3sEd2qq15pVOteI3jMMrMDqQKjwgpJFg8yV5RGulgQ2lToOAmUy1WDgXN9HjaZmaRkRn1KE0sygsoQAYAOdjk+ZNCUjSgpQKUpQKUpQKUpQKUpQKUpQKUpQKUpQKUpQKUpQKUpQKVgmo1r15ji3xp8ZmGU94jG3MPv9X3k7UHZeXkcS6pHCjoMncnyUdWPuG9cXpU8nsoxGv25wcn4Qrg/rFT7q32nDUQ6yS8hGDI5y+PIeCj7qgD3V2AUEb+Z1f2zvN5hm0x/DlJhSP0s/Gu+C3SNdKKqqOgUAAfIVspQYxQ1mo/jt20MDumNfdRM9OZIwRM+7UwoO5XBzgg4ODg9D5Gs5qqwMYGEUWpliZYgucG4uZF1yPK2NwqnUffnyArkv/wAokUN1JamCQmO4trcsCuC06FwwBPQY/bQXXNY1DOM7+Xw61SuzP5RI7+SOKK3dXlhnlXWy4HKkMZUkZ6kda3XMzuCykmRFa6tWONQ0nE9s5GxGe78GHiuaC4VnFarScSRrIvRlVh8GGR++ttBpurWOVdMiK48nUMPwNcJ4UU3glkj+6SZI/gUckge5StSlKCKN7NH7aLK/zkILj4tH66/LV8a7ba4SRdaMHU+KkEe8ZFdFcFzwxWYyIxik+2mO9jpzF6OPiM+RFBo45xqO0EbSKxV5ViyoBCagzF3yRhAFJJGcDwrNvxyBh3nWM82SJRIyqzNHJyzpGdwWxj9Iedcl9CZWjjuEOY2MmY11Ryry3jYEHdfa7g/Ineou37LoiqgeZgOcrcxJCHjlk16W0Muojpk5yOozU/KSn7Xj1rIHInTEZl16mUaREdMjHJ9UHx6biuifilvGCXmiUA6SWkUANpD6Tk7HSQ2PIg1BvwRTsTJpFxJNjlblZGLSwsT1RifIbAdetePzOsca4kkVlWZGkeLOpZtIbIOBqASMA/d3BzSBNXXF4lOlXR2EkUbqrjUnMYKCw8OoO9dsEySKHRlZSMqykEEeYI6iquOF2skL2oZ2R5EkKgEthFjXTkHJ2jGT76nuESDRy9ZdogqMWXQx7oKkgbZKkdNs56dKqu7FMVmlApSuc3sQ2MifrD/Og6KVoW8jJwJEJPQBgT+FbgaDNKUoFKUoFKUoFKUoFKUoFKUoFKUoFKUoFeZHCgkkAAEknoAOpJrJqMb/AHiQj+RjYg+Ukqn1feqnr5sPunIAjXO7ZWHwToZR4F/JPu+Pj5VJhQNsbVmlApSlApSsZoM1Edqji2Mh6RyQSt+hHKjOfkoJ+VSF1dpEjSSMERRlmYgAD3k1qtJufGWaMqrZAWQd5kIxl0Pq537p3x1wdgEBI/KnYkEiK457Y3JhniKcwDxCsTnyC1Qu0XZ67PEricQNym4hw6VZCVCGNImVmyT0BIHzFfUuHcHEYQMxcxFxC5yHETDHLc574HTfrpU9RmtfFYxNcwW5AZV1zyAgEEINMYIP33z/AGKD5l+THgNza3NtPPC0cSWl4HkbGka7kuu4Piu/wq728ojw7bCGC8nkznui4k1xoffpU7e4VL8BAj51tjaKQ6B4cuX6RMDyBZl/s1x8XtY49CkSmNpGmlWOGWaSZwQUDuinCggbHqFUDYUEtwG3MdrDG3rLFGp+IUZFd9V2btLJ/JcOvJdtjoiiHz50it+yta8X4g3SwCbba5o2PzAI/YTUmaFlzQGqheXfElCmTlIGdVAjI5h1HfAYSKcDLHpsprbwfikayyma9BAxGqyyRKSwOZH0gLjfu/2Wrm/l4e9GlN3V+OmtvFrXSoKftfYxnDXKZH2ct/CDWods7VvZrcS++K0uHHzITArpibZTD+2X+rk/ijqN4h2mgit2uAS4VrlAoGC0lsJTKgJ2HsXGTgbDzrdaX5lnX6GVBy3IMihc96PYDOr8QKrPCF2tv+bcV/jv6o7+yHG5Z7i4ilYNpw4xjCkTTW7qo6hSbfWMkkF2XJ013dt4RJaaNAk1S245Zx9J9KvdGdvx223wM128L4PFbl2jBGsrnOO6qjCovkuSzfF2JO9QvExcm9Gz6FOuJyYfR0HK0ENleZzCzNjBxg/EUEF2U4WIr9JOQsYL3aK/c6plTCCpySfWGeojY+GKu/Dv/EXP6cX+ClU9VuxCSY5+7MjmPNr6Q7gS8ySHu6CpcxMM74D9OlWXswJBzBMcy4g5h2Pe5CZ3AA6+QxQTtKUoK12t7Ri3HJimjS5IjdRLFLIgiMml3KxDJ2V8DIyQK+eJw21BZmk4c7O8kjNLwi7dizsWbvPKTjJ6eFXq+uGj4pcSLoyvDrcjmMVTPpFx6zKrED3gGuNu1t15cO/vlx/+Wgph4LFO8UXJtkguGiNrxPh0CQsk662MRUyM49mRqGMEYI8vqfYy+efh9rPKdTyW8Lu2AMsyAk4G1fO7Oyjt+JPbwKI4Vv8AhrJGrExqXsbguVB88DJ8cCrB2YuZRY8LhjMgD2as3KEJfuJCAfphjSNZzjfpQX6lVLi/amWOOYxwLlBc8tnk7rNbkBtSquQDnbB8MbbGuqLjcgnaHl6mMoTGsBIwLaOV8MF1MO8cbZJI6DpLFjpUH2c7QelqX5MkalI5UZlkCskmrA1OijWNOSF1Aal7xzXJb9rCyBuTpZ44JIl1ltSylwA2hGYMOWx0qrbb+DaaLPSqwnaVp4WeKPSOQJCzOAyM8blNKEd7BXckjrsDggaIu2H0UbLBJIXWVgoDmQrCIxJmNEYo5Z9kbHvK9KC3ZpVeteOuI7qWWNQsMrKgRjqZQqkaw4AU94ePj7t9N1xaeR44VQIy3QhnxKcY5BmHLfl5YEFc5CnII6b0FnpUJN2gVbtLXRkM3L1gsdMnKaXDYXSO4vQtq3B04Oa5bntUVafTDrSBJ2YiTv5hC7FNOF1FjpJbfS3TFBZaVAxdoG5ojaEACVIHZZNRErxiQYUqNS6WUatjknbAzWv/AGnyE0xZaRIGUFwADM7IoY42AK7nB8cA4xSxYqVx8KvufEJNOk5dWXOcNG7I2D4jKnB8RiuygVpuLhUGpmAHv/b+zf3AE1taqlBFNIhlc80rLcoSAASsdydA0DriNHXzPM+OCWtU6lkIVtJIIDYzgkbHHjjrXizgWNFjUYCgAZ67eJPiT1J86zaKVRFbqFUH4gDNbGorOazmqLe308Mz96UpaSvK47x5sVww0L46wgeTbw5a+6sSrcgpCTIz67LXmeeMMzxztIC6E6F1AeqPAA+FQXrNM1C8ALa7lWB7syKuXkfb0eEnDSHpknoBk5JGSSfXF+M8thDEvMnYd2Mb6V+2/kvxxk+I6hM1FpDo4pxLlDSo1yEEomcf2nP1V9/4ZNaJ+OxxwRzNnMiIyxru7FgDhR8wM+8eYqFurB4Ghmkd3keRkkCZKnVFIVXAxq7yKASABnYKKkuz/Z9YQJJMlwoVAzauWoGANXi2Ns9Bk46ktw4auvOtljONY1ExPx3fnrpuoq2LDhUk0q3V5jK7w24JMUX32H8pL94+rvjG9SnF+Ix20LzyZ0oM4UZZidlRR4szEKB4kiuhZQSVBBK4DAEZBIyMjw2Oarkn++3yrvyLRtZ+zLckEID4MI9zj7WPIV3XDMpnhMcoiXnHMjZZwDlVLHOhT5LsoPjjPjUALl3vG0yCJZWeNZMAkragApHq7uoySSnJzsnTysXFLzkwySkZ0KzAeZxsPmcCq/w3hj3EAhdgsUZZDhFZ5ZVP0khMgIUczXjAycZz4VUZ4bcv6YQzhwxe2MqjAdo1E0fTbUoaVTg9V+IHXc3Vxad5le4tx1ZBm5iH3kHtlA8V74x0brXBxeKS1jSMEOqtzISEVHV4QZTGQgCsGRZBkAEE75zmrVFIGAZTkEAg+YIyDUGrh9/FPGJYZFkQ9GU5HvB8iPLwrpqBv+z2HNxatyJju2Nopf61AME/exnc1iw7QANyrleTIMZJ9mc9Dn6oJ8TkeAYnasZakYzy1Tt4lwpZ3R2eReXrwEYAHUACTtnIAIBBHU+dak7N231kZ/H6SSRwfirHH7KlwazU9nCct+2L+a5+y3JbcNhj9nDGn6KKv7hXXilK9IikQHauGR4ZEhDFzDJhUbSzDXHqVWyMErkdR16iq0eDQ+HDJBhnZfoT3Wbmd7a5695ckYz3/td29XdikpBbUCM4KSSRnfGRlGGRsOvlWj8zR/am/vNx/qVRULjhkRQhOHXAfJwxU6cathjnn6u2fMeHSpDgvAubDPHJHNDEbhXhR2UsFEEasQGLhVL6zjz38d5/8zRfam/vNx/qVH2nDEFzNEXnIKxSp/vVxsGBRlH0ngYtX/uVJxieJSuXa/BkM0c2W1RqqqO7jA1dds/WPQjwr1w729z+nF/gpWfzNF9qb+83H+pXRZWKRatGrvEFizu7EgADLOSegFIiIWqdNKUqik8efTf3Zzj/APm2+/Mkj/8AM3H8pCC6fFQTVS9Mb+dP/duM/wChX0LivZ6WS69Lgu2gYwpCwEUbhlR3cHv9DmQ1r/Md/wD+qP8A3aD/ACoKVfORxiZV08w3vDdCsxwzCwuerYzjJGTjx6VeuAdnUSxtba5iileCGOPJQOoZUCsULjODjyFY7O9mmtpri4kuGuJJzCWZkRdPKVlXSF26H9lWGg5ZOHxMCDEhB15BRSDzPaZ231ePn41mGwiQgpEikdCqKCO6E2wNu6AvwAFdNKDlteHwxFjFEkZbdiiKpY741EDfqevma4rLs9axwiAQRsuE1ao0OsoO6z7YZh51L0oOAcJt8qeRFlVKKeWmVUggqpxsMMwwPM+dJuEW75D28TZIY6o0OWChATkddIC58gBXfSiU5Rw+LMh5SZlxzToXMmBpGs473d238Kzb8PhjVVSJEVSWVVRQFY5yygDAJ1HceZrppRXG3DYTLzjDGZB0k0LzBtjZ8Z6Ejr41Hxdm4hM8x1Nr5mpWEZVhIe8GOjW6+SsxA8thicpQcnoEXM53KTm40iTQvM077a8Zxuds+NctxwC2YY5MaguruFjQcwglu/t3gSSTnzqVpQa7aFY0CIoVVGFVQAqgdAANgK2UpQeJxlWA6kH91RXZ22kRGEgwSwx06YHlUxWMVUmObR/EbCRzzIp2ikAwPrxHGcB4jt49VKn31FP2gntiFvrchf6Rb6pIPcXT2kWfeGHhqO2ZcXDC6MRPdaIOg8dSuRJv8Hj/AANe+K2ryxFI5DGx+sBnbxU7g4PTIIPkazl0rNjfRTLrikV181YHHuOOh9xreZF1aMjUQSFzvjxOPKqVaxS8PlMs1oZEAKJLaguYotjpaAAOemSw1H9uNfa+OG8sru8gudSray7xvhleGOYgHxQ/SEEHBGCPE1jSyyywic4qfhZWbiF3IWMFsBzMAvIwykKnOCR9dzg4UeW5Hjt4LwaO2UhdTux1SSyHVLK32nb9wAAHQACuHsvwbkKja2YcmNRqOWPdXJdvE7D5lz9bAsFeiMaa5uIXawxtK3RR0HUnoqjPiSQB8a6jVY4sfTLtbMeyhAkuD5swPLj+JGSQfBs9QKzlcRx2IeC7kVHuIvav3JyQAWZ8NFLGD62zaUB6h1Bxp2tnAeHC3gWPbV1cjfLnrudyBgAZ8AKT8HR7iO4JIKKRoGNDEezZh4lMtjw7x8hiRArl9L6fPR3Rnlu54v8AETzz5vxS5TaJ4335ILf7cnMf9CDD/wAfLHzrjN76G7r3Hid2dQJI1kjZzl1KuwDLqJIIORnGK6b3ghmuTLJIeUIlRYkJUklizl3G5BwndGOm+a6obO2iIRY4kJ6AKgJ+WMmuxlH2c/pcySMUWOPUyR8xHkZ2UrqcISFAVmwMnJbO2MV29nX+g5ZGDEzwkeWhiE/FNJ+dep+HW0uQ0UTEde6upT8RuprXwbhLW8sx5peOQxsofd1KrpYF/rDAXBO+25NFSp6VSuMSyghrpCWVvoUjwITk6ciQjUTpPeU4yMgI3jdq1zQK4KsAynYqwBBHkQetcvq/T+/pzhcx+v8AdfTWM1Nojs3asil+arRtjRHHkxx4znSzb+7HdAx6o3qaFVy44LLAxls397QSElHx5N4HHTPkoyAMHs4Px+OdjEQYp1GXgk2kA6alH10z0YbdOleuljtxjGuvP9/aSmKV4klVRqYgDzJwPxNaPzhD/Ox/rr/nXqjqpXlHBGQcg9COleqBUaTi9AHjA2flIuP4mqSqMt21Xkp+qkUSf22Luw+S8s/2qCTpSlApSlAzSqV+VGWQQ2aRyyxc3iNnC7QyPG5jkLq660IOCD+wVyXFrwuN2jk41MjqSrK3GJFZWBwQymXII8jQfQKV8n44ugLPwfi4mlg5k0sU/EZLhJIUjJYckM2r9mNtwcGvo/Z7iPpVpBdadPOijl05zp1qGxnxxmgkaUpQKUrFBmlYrNApSsZoM0pSgUpSgUpSgwai7vjKrtGRId91IK5DaSNQ8QxVSPAsPfUnI2AT5DNVfgnDkeJ9GEIluGGkDSBLMZlyB450E/o4okzzSyRhG0ygAnT3WxuFbBIB9+B+A8qi+M9pra2YRtIGlOMRIQX7xwuoZwoJ8TitUvCbiZRHJOYYgAojtiRIyjbD3BGoAjHqBSPtGuSz7Pwwq9ki8nUwlSVAvMkCur4d3B1upUAlskrpOSc4zne2dvaw9p2xhVMye2LyqLeDM0p0SMgIVRncr1IA99U7txw9+I3FpatElo1y5aQhgbxraJGLCXR3AuTgKS41HPgRU+eJw2c7i3jDLGCJyq6ppJG6HmeIV9KkeHMPTTvyDgt3DeRcUaA3MrRTLPGjqJIzI0RjSPmEKURUK4yN2Y76jXno6m68ZnmO/wBrKZ/J4nKhuLTLEW13PEus6m5baZYhk+SSqPlVrqr9krOcTXd1NDyFuGhZImdWkBRCrO5TKgkaRgE+rUpxjjSW+lNLSTPnlQR4Mj4xk4OyqMjLnYZ617I38WvuRE0mnURgIg6u7HCIPixArm7N8J9Gh0uweV2aWeQDGuZzliPuj1VHgqgVEycNvWxcG4iNyp1LbHe0UYI0A45mvBI53mfVx3a8R8T4jG9sbkWyrPNyjEiSGSLKSMv03MKuQEGe6OtBb6VgVmgwap/H+FyvdO6I5yljy8KhjZ4Z5HIkY99AMqcqR1+t0q40oK92ds3imlUIVhPeUyIgl5jSSM660P0iDVkE7jUd2ztYaUoFKUoMEVwcW4PDcheYveQ5jkUlZY2+1HIN1Pw69DmpClBUuM8IacQ2d4FuIWl9c91n0xSELKi4GcgHUuxwe6u2Tfk34WUEZtF0AkhdT6QT1IGcZqb4r7W2/rz/AIE1SNBEdlLVIbbkxjSkcs6Iu/dVZnCgZ8hUxUbwH2b/ANfdf471Ik0HieZUVnYgKoLMT0AAyT+Arj4JERGZHGHlYyuPEagAqn3qiov9mtV1/vEnKHs0ZWlPgzDdYvf4M3u0jfUcSgFBmlKUClKUFJ/Kl6nD/wDmvD/4mrm4rfSCeUCS6ADuAE4PJKowT6soQ8wfezv1rp/Kl6nD/wDmvD/4mrVxLhFw00jLb3jAu5BTi80akEndYw+EHkvh0oOT0qR4LoNJcMPRLraXhj2y+yP8qyjf3eNdXBL2WPhfCkiZxzII1PLVGc6bRnUKJBp9ZR18PEVqfhs0cF00kNyg9EuhmbiMlymeWduUzEZ697wx76newUKtwqwYqCVtbcqSASpMIBIPhsSPnQa5e1RhKxzplgjc0xamCyx25nkQDTp9VTgF9W42wc1lePSLeJBImDJHHpUOGjB1OWYy6R3tAGFxkkHGQCwnDwyAyc7kx8z+c0Lr9XT6+M+rt8Nq1w8Gt0XSsEKr3e6IkA7h1LsBjYgEeRFBAXHayUxs0NufaIqNJzVjI9JSFwzcvAY6sqFLeJPTBk+JccMLleXq5ccckxD40pI7IvLBH0hyjZHd2A8Tiu1uEW515gi+k9p9GnfycnXt3t9969y8NhYozRRs0fs2ZFJTcHuEju7qvTyHlQQ8HFJIzeTTDKxSqkaq+dhHHtgqoXLPkkk9T4AZ13Xal4+Ypt8vEJ2lAlGgLFHFIdDlQXJSZcAhdwQcdan2s0IZSikPnWCow+QFOofWyABv4CtUfCoFXQIYwuJF0iNQumQ5kGAMYbAyPHG9REenaDVdG3WGQqrrG0gVyFdohKCSEKBNLAZLA5PTG9ezxrv6BGSS86AAjJ5K6jgHHXIHXxG9SB4fEZBNyo+YNhJoXWBjGA2M9CR868HhkGtpOTHrb1n0Lqbu6e82Mnu7b+G1VaQUfa0khBAzOBqkRBMzxqSAAU5IYPg50sFGMEMc1NcH4g06s5j0KJJUXLZZuXI0ZYgDCglcgZOx3wdqDgtthR6PD3SSv0Sd0kgkrtse6PwHlXZDAqDCqFGScKABliSxwPEkk/Og2UpSgUpSg8yLkEeYIrj4Xw5YFKgk5Od8eWPD4V3UolRdvBkUeI8PH8Kr/G+IM8aRQqPSZHZY87iEocSTnH1UByPtF0X61eFs45Ly4IwrMIe8AMlodY7w8gWU+GcVKcK4UISzk6nbbJ+qg9VF8h4nzJPuwIm0fYcIto2igKtrhJdC53mJALyEjZzqAJHgVU4AxVhArRd2iyLpYeOQRsysOjKRuCPOuMXbwbTZaPwmA6D/AIyj1f0x3euQtSIiFe+OvcrbubREefGI1lYrHknGWI3wBk42zjGR1qlcOgd42uMPraSWG4ZiXleSKRk0kxEFo8+rGpjRRrLGvoccoYBlIIO4IOQR7j41XOxqK0FwrAEG84gCDggg3D5BB6iqK6IofW0rgnAOi0K6viIghOfATavfmujiSycmJTzGLzqtsQ5R4pgrkSK0wLIAFcGOQP12JFTCcCf00seT6LyxjvSekmbVuGYnBi07aOmMDFbO1AAksAMYF6mAPAciagkuAPdGBfTFjWYZDcpiUIHRhnpny3x51JUFKBSlKBSlKBSlKBSlKCJ7ScBivoeRK0ijUHDROUdWGRkMPcTVcf8AJjbGIRelXmlWZgRMofLAA5k0amG2wJwKvNap7hUUu7BVG5ZiAAPeTQcvBuGJawJbxliqAgF21OSSSSzHqSSa83F0zsYYT3h68mMiIY6eRk8l8M5O2NWvmyXGyaoovFyMSuPuKfUH3mGfIDZqkLa3WNQqDAHQfvJPiT1JPU0GLW2WNQijAGfeSSckk+JJJJPiTW6lKBSlKBSlKCK7QcBhvUSOcPhJEmQo7RssiA6WDoQQRknr1xXB/sbF/Sb/AP7hd/6lWSlBWJexEDKVa4vmVgQytf3RDAjBBBk3BHhU7wyxS3hjt4wRHEixoCSSFUAKMnc7CuqlApSlApSlApSlApilKBSlKBSlKBSlKBSlKCNteF6J3m1Z1atsdMkHr8qkqUpaRER0VjFZpRUZJwvSxeBzExOSoAaJj5vHtv71Kk7ZJqLk4dbKzNc8PhyzFmljgSVGJOSzYXWCepJGPvGrPSghLbgnDpF1x21q6/aWKJh+IFdMHALNHWRLWBXU5VlhjDKcEZVgMg4J6edbLnhMMjayml/5yMmOT/qIQx+GcVpNlcJ7O41D7M8Yf5BkKEfE6qCUpUZ6Vcr61urj/gyjV+rKFA/WNDxkAd+G4X3clnx8TFqFBJ0qMPH7cetJo/rEeP8AjUU/2hs/6VB/1o/86CTpUZ/tDZ/0qD5TIT+ANBx63Pqu0n9VHJJ/hqaCTpUZ+eMjuQXD+7laP8UqKekXT+rCkY85ZMt+pGCP/lQSdc95exxDVI6oPDUQMnyA8T7hvXGOHzP7W5P6MCCJfmWLPn3hhW+04XFEdSoNR6uctIfjIxLH5mg0enTSbQxFR/OTgovyi9dvgdPxrZbcMGoSSsZXG4L40of+HGNl+O7Y6k1IUoMYrNKUClKUClKUClKUClKUClKUClKUClKUClKUClKUClKUClKUClKUClKUClKUClKUClKUClKUClKUCsEZ2NKUGRSlKBSlKBSlKBSlKBSlKBSlKBSlKBSlKBSlKBSlKBSlKBSlKBSlKBSlKBSlKBSlKBSlKD//2Q==" 
            alt="5G network infrastructure diagram" 
            className="tech-image"
          />
        </div>
        <div className="col-md-6">
          <h3>The Mobile Revolution</h3>
          <p>
            The evolution of mobile communication has been particularly rapid:
          </p>
          <ul>
            <li><strong>1G</strong> (1980s): Analog voice calls only</li>
            <li><strong>2G</strong> (1990s): Digital voice and basic data (SMS, MMS)</li>
            <li><strong>3G</strong> (2000s): Mobile internet access and video calling</li>
            <li><strong>4G</strong> (2010s): High-speed mobile broadband</li>
            <li><strong>5G</strong> (2020s): Ultra-low latency, massive device connectivity, and enhanced mobile broadband</li>
          </ul>
        </div>
      </div>

      <h2>Current Communication Technologies</h2>

      <div className="row">
        <div className="col-md-6">
          <h3>5G Technology</h3>
          <p>
            The fifth generation of mobile networks offers unprecedented advantages:
          </p>
          <ul>
            <li><strong>Speed</strong>: Peak data rates up to 20 Gbps</li>
            <li><strong>Latency</strong>: As low as 1 millisecond</li>
            <li><strong>Density</strong>: Support for up to 1 million devices per square kilometer</li>
            <li><strong>Reliability</strong>: 99.999% availability</li>
          </ul>
          <p>
            5G employs advanced technologies like massive MIMO (Multiple Input Multiple Output), 
            beamforming, and millimeter wave spectrum to achieve these capabilities.
          </p>
        </div>
        <div className="col-md-6">
          <h3>Satellite Internet Constellations</h3>
          <p>
            Companies like SpaceX (Starlink), Amazon (Project Kuiper), and OneWeb are deploying vast 
            constellations of low Earth orbit (LEO) satellites to provide high-speed internet access globally.
          </p>
          <p>
            Unlike traditional geostationary satellites, LEO satellites:
          </p>
          <ul>
            <li>Orbit much closer to Earth (500-1,200 km vs. 36,000 km)</li>
            <li>Provide significantly lower latency (20-40 ms vs. 600+ ms)</li>
            <li>Require constellations of thousands of satellites for continuous coverage</li>
            <li>Can provide service to remote and unconnected regions</li>
          </ul>
        </div>
      </div>

      <h2>Advanced Communication Paradigms</h2>

      <div className="row mt-4">
        <div className="col-md-6">
          <h3>Quantum Communication</h3>
          <p>
            Quantum communication leverages the principles of quantum mechanics to enable secure information 
            exchange. The most notable application is <span className="highlight">Quantum Key Distribution (QKD)</span>, 
            which allows two parties to produce a shared random secret key known only to them.
          </p>
          <p>
            The security of QKD is based on the fundamental properties of quantum mechanics:
          </p>
          <ul>
            <li>The no-cloning theorem prevents perfect copying of an unknown quantum state</li>
            <li>Measurement disturbs the quantum system, alerting parties to eavesdropping attempts</li>
            <li>Entanglement enables correlations that cannot be explained by classical physics</li>
          </ul>
        </div>
        <div className="col-md-6">
          <h3>Internet of Things (IoT) Communication</h3>
          <p>
            IoT requires specialized communication protocols to accommodate a vast number of 
            devices with diverse requirements:
          </p>
          <ul>
            <li><strong>LPWAN (Low-Power Wide-Area Network)</strong>: Technologies like LoRaWAN and NB-IoT provide 
            long-range communication with minimal power consumption</li>
            <li><strong>Mesh Networks</strong>: Protocols like Zigbee and Thread enable devices to form self-organizing, 
            self-healing networks</li>
            <li><strong>5G mMTC (massive Machine Type Communications)</strong>: Supports extremely high device density 
            for industrial and urban IoT applications</li>
          </ul>
        </div>
      </div>

      <h2>Communication Security</h2>

      <div className="row">
        <div className="col-md-4">
          <div className="feature-card">
            <i className="fas fa-lock fa-2x mb-3" style={{ color: "var(--primary)" }}></i>
            <h3>Encryption</h3>
            <p>
              Modern systems use advanced encryption standards (AES), public key infrastructure (PKI), 
              and forward secrecy to protect data in transit.
            </p>
          </div>
        </div>
        <div className="col-md-4">
          <div className="feature-card">
            <i className="fas fa-user-shield fa-2x mb-3" style={{ color: "var(--primary)" }}></i>
            <h3>Authentication</h3>
            <p>
              Protocols like OAuth 2.0, SAML, and multi-factor authentication verify the identity 
              of communicating parties.
            </p>
          </div>
        </div>
        <div className="col-md-4">
          <div className="feature-card">
            <i className="fas fa-shield-virus fa-2x mb-3" style={{ color: "var(--primary)" }}></i>
            <h3>Post-Quantum Security</h3>
            <p>
              As quantum computers threaten current cryptographic methods, post-quantum cryptography aims 
              to develop algorithms resistant to quantum attacks.
            </p>
          </div>
        </div>
      </div>

      <h2>The Future of Communication</h2>

      <div className="row mt-4">
        <div className="col-md-6">
          <h3>6G and Beyond</h3>
          <p>
            Though still in research stages, 6G aims to provide:
          </p>
          <ul>
            <li>Terabit-per-second data rates</li>
            <li>Microsecond latency</li>
            <li>Integration of sensing, communication, and computing</li>
            <li>3D network coverage including undersea and aerospace</li>
            <li>Native AI integration for intelligent networking</li>
          </ul>
        </div>
        <div className="col-md-6">
          <h3>Quantum Internet</h3>
          <p>
            The quantum internet would connect quantum devices, enabling:
          </p>
          <ul>
            <li>Secure distributed quantum computing</li>
            <li>Quantum sensor networks with unprecedented precision</li>
            <li>Unhackable global communication networks</li>
            <li>Distributed quantum clocks for ultra-precise timing</li>
          </ul>
          <p>
            Early quantum networks are already being tested in several countries, laying the groundwork 
            for this revolutionary technology.
          </p>
        </div>
      </div>

      <div className="text-center mt-5">
        <Link to="/" className="btn btn-outline-primary btn-lg">
          <i className="fas fa-arrow-left me-2"></i>
          Back to Home
        </Link>
      </div>
    </section>
  </div>
);

export default Communication; 